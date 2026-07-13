#!/usr/bin/env python3
"""
Teleop stereo-camera recorder — stores episodes as LeRobot dataset v3.

Records two cameras and the trigger position simultaneously.  The trigger
mirror loop (reader → normalize → gripper) runs in its own thread; the
normalized trigger value is snapshotted at every camera frame and stored
as `action` in the parquet.

Controls:
  R  — start recording a new episode
  S  — stop and save the current episode
  D  — discard the current episode without saving
  Q  — quit (writes final dataset metadata)

Usage:
    # Feetech trigger + DM4310 gripper (typical SO-100M setup)
    python teleop_trigger_record.py \\
        --reader feetech --leader-port /dev/ttyACM0 \\
        --gripper dm4310 --can-port /dev/ttyACM2 \\
        --raw-min 1676 --raw-max 2236

    # Camera-only (no trigger hardware)
    python teleop_trigger_record.py --no-trigger
"""

from __future__ import annotations

import argparse
import fcntl
import glob
import json
import os
import queue
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

# Import trigger/gripper abstractions from the sibling module.
from teleop_trigger import (
    DM4310GripperController,
    FeetechGripperController,
    FeetechTriggerReader,
    GripperController,
    MirrorConfig,
    SerialTriggerReader,
    TriggerReader,
    normalize,
)

# ── Camera auto-detection ────────────────────────────────────────────────────

def find_capture_cameras() -> list[int]:
    """Return the /dev/videoN indices that are real video-capture nodes.

    Each UVC camera exposes several /dev/video* nodes; only some are capture
    nodes (the rest are metadata-only and cannot be opened for frames, which is
    what breaks a hardcoded index list after the cameras are replugged). This
    queries each node's V4L2 capabilities and keeps the first capture node per
    physical camera (grouped by bus_info), sorted by index.
    """
    V4L2_CAP_VIDEO_CAPTURE = 0x00000001
    V4L2_CAP_DEVICE_CAPS = 0x80000000
    # VIDIOC_QUERYCAP = _IOR('V', 0, struct v4l2_capability) — struct is 104 bytes
    VIDIOC_QUERYCAP = (2 << 30) | (104 << 16) | (ord("V") << 8) | 0

    first_per_bus: dict[bytes, int] = {}
    nodes = sorted(glob.glob("/dev/video*"),
                   key=lambda p: int(re.search(r"\d+", p).group()))
    for path in nodes:
        idx = int(re.search(r"\d+", path).group())
        try:
            fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            buf = bytearray(104)
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf, True)
        except OSError:
            continue
        finally:
            os.close(fd)
        caps = int.from_bytes(buf[84:88], "little")
        device_caps = int.from_bytes(buf[88:92], "little")
        effective = device_caps if (caps & V4L2_CAP_DEVICE_CAPS) else caps
        if not (effective & V4L2_CAP_VIDEO_CAPTURE):
            continue  # metadata-only node
        bus = bytes(buf[48:80]).split(b"\x00", 1)[0]
        first_per_bus.setdefault(bus, idx)
    return sorted(first_per_bus.values())


# ── Defaults ───────────────────────────────────────────────────────────────────

DEFAULT_CAM_IDS = [0, 2]
DEFAULT_FPS     = 30
# These cameras are side-by-side stereo: the resolution is the COMBINED frame
# (both eyes share the width), so valid modes are double-width like 2560x720
# (1280x720 per eye) or 1280x480 (640x480 per eye) — NOT 640x480. The old
# 640x480 default isn't a real mode, so the camera silently fell back to
# 640x240 (320x240 per eye). List your camera's modes (e.g. `v4l2-ctl
# --list-formats-ext`) and set --width/--height to one it actually advertises;
# whatever the camera really delivers is verified and used (see CameraCapture).
DEFAULT_WIDTH   = 2560
DEFAULT_HEIGHT  = 720
DEFAULT_OUTPUT  = Path("./lerobot_dataset")
DEFAULT_TASK    = "teleop recording"
CHUNKS_SIZE     = 1000
VIDEO_CODEC     = "libx264"
VIDEO_CRF       = "18"


# ── Frame container ────────────────────────────────────────────────────────────

@dataclass
class Frame:
    timestamp: float
    images: list[np.ndarray]   # one BGR image per camera
    action: float              # normalized trigger position [0, 1]


# ── Trigger + gripper thread ───────────────────────────────────────────────────

class TriggerGripperThread:
    """
    Runs the trigger-mirror loop in a background thread and exposes the
    latest normalized trigger value so the capture thread can sample it.
    """

    def __init__(self, reader: TriggerReader, gripper: GripperController,
                 cfg: MirrorConfig) -> None:
        self._reader  = reader
        self._gripper = gripper
        self._cfg     = cfg
        self._lock    = threading.Lock()
        self._action  = 0.0
        self._running = False
        self._thread  = threading.Thread(target=self._run, daemon=True,
                                         name="trigger")

    def start(self) -> None:
        self._running = True
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        self._gripper.close()
        self._reader.close()

    def latest_action(self) -> float:
        with self._lock:
            return self._action

    def _run(self) -> None:
        period    = 1.0 / self._cfg.rate_hz
        smoothed: Optional[float] = None
        last_sent: Optional[float] = None
        ramping   = True

        while self._running:
            t0 = time.perf_counter()
            try:
                raw      = self._reader.read_raw()
                pos      = normalize(raw, self._cfg)
                smoothed = pos if smoothed is None else (
                    self._cfg.ema_alpha * pos
                    + (1.0 - self._cfg.ema_alpha) * smoothed
                )

                with self._lock:
                    self._action = smoothed

                moved = last_sent is None or abs(pos - last_sent) > self._cfg.deadband
                if moved:
                    last_sent = pos
                if moved or ramping:
                    ramping = self._gripper.set_normalized(smoothed)

            except Exception as exc:
                print(f"[trigger] warning: {exc}")

            elapsed = time.perf_counter() - t0
            if elapsed < period:
                time.sleep(period - elapsed)


# ── Camera capture thread ──────────────────────────────────────────────────────

class CameraCapture:
    """Reads all cameras in a background thread; snapshots trigger action per frame."""

    def __init__(self, cam_ids: list[int], fps: int, width: int, height: int,
                 trigger: Optional[TriggerGripperThread] = None) -> None:
        self._interval = 1.0 / fps
        self._trigger  = trigger
        self._lock     = threading.Lock()
        self._latest: Optional[Frame] = None
        self._running  = False
        self._recorder = None      # active _StreamingEpisode, or None when idle
        self._caps: list[cv2.VideoCapture] = []

        actual: list[tuple[int, int]] = []
        for cid in cam_ids:
            cap = cv2.VideoCapture(cid)
            # MJPG unlocks the high-res side-by-side modes on USB stereo cameras;
            # without it many are stuck in a low-res uncompressed (YUYV) mode.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            if not cap.isOpened():
                raise RuntimeError(f"Cannot open camera {cid}")
            aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual.append((aw, ah))
            if (aw, ah) != (width, height):
                print(f"[warn] camera {cid}: requested {width}x{height} but got {aw}x{ah} — "
                      f"camera fell back to a supported mode. Set --width/--height to a "
                      f"resolution it advertises (stereo modes are side-by-side, e.g. 2560x720).")
            self._caps.append(cap)

        # The writer and dataset metadata use a single resolution, so all cameras
        # must agree. Use what the cameras ACTUALLY deliver, not what was requested.
        if len(set(actual)) > 1:
            raise RuntimeError(f"cameras returned different resolutions {actual}; "
                               f"set --width/--height to a mode all of them support.")
        self.width, self.height = actual[0]

        self._thread = threading.Thread(target=self._run, daemon=True, name="capture")

    def start(self) -> None:
        self._running = True
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        for cap in self._caps:
            cap.release()

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._latest

    def set_recorder(self, recorder) -> None:
        """Route every captured frame to this episode (None to stop recording)."""
        self._recorder = recorder

    def _run(self) -> None:
        next_tick = time.perf_counter()
        while self._running:
            now  = time.perf_counter()
            wait = next_tick - now
            if wait > 0.001:
                time.sleep(wait)
            next_tick += self._interval

            images: list[np.ndarray] = []
            ok = True
            for cap in self._caps:
                ret, frame = cap.read()
                if not ret:
                    ok = False
                    break
                # OpenCV reuses the capture buffer on the next read(); copy so a
                # recorded frame can't be overwritten mid-encode (frame tearing).
                images.append(frame.copy())

            if ok:
                action = self._trigger.latest_action() if self._trigger else 0.0
                frame  = Frame(timestamp=time.time(), images=images, action=action)
                # Feed the recorder straight from the capture thread so a slow
                # display loop can never drop recorded frames.
                recorder = self._recorder
                if recorder is not None:
                    recorder.add(frame)
                with self._lock:
                    self._latest = frame


# ── ffmpeg-backed video encoder ────────────────────────────────────────────────

class VideoWriter:
    def __init__(self, path: Path, width: int, height: int, fps: int) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._proc = subprocess.Popen(
            [
                "ffmpeg", "-y",
                "-f", "rawvideo", "-vcodec", "rawvideo",
                "-s", f"{width}x{height}",
                "-pix_fmt", "bgr24",
                "-r", str(fps),
                "-i", "pipe:0",
                "-vcodec", VIDEO_CODEC,
                "-pix_fmt", "yuv420p",
                "-crf", VIDEO_CRF,
                str(path),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def write(self, frame: np.ndarray) -> None:
        assert self._proc.stdin is not None
        self._proc.stdin.write(frame.tobytes())

    def close(self) -> None:
        assert self._proc.stdin is not None
        self._proc.stdin.close()
        self._proc.wait()


# ── Streaming episode recorder ──────────────────────────────────────────────────

class _StreamingEpisode:
    """One recording in flight: frames are encoded to disk as they arrive.

    A dedicated writer thread pulls frames off a bounded queue and pipes them
    straight to ffmpeg, so only tiny per-frame metadata (timestamp, action) is
    kept in RAM — recording is O(1) in memory regardless of episode length.
    Videos stream to a temp dir and are moved into place at finalize, once the
    episode index is known. If the encoder can't keep up the queue fills and
    frames are counted as dropped rather than ballooning memory.
    """

    def __init__(self, dataset: "LeRobotDatasetWriter", uid: int) -> None:
        self._ds  = dataset
        self._tmp = dataset.root / "videos" / ".pending" / f"ep_{uid:06d}"
        self._tmp.mkdir(parents=True, exist_ok=True)
        self._writers = [
            VideoWriter(self._tmp / f"cam_{i}.mp4",
                        dataset.width, dataset.height, dataset.fps)
            for i in range(len(dataset.cam_keys))
        ]
        self._meta: list[tuple[float, float]] = []   # (timestamp, action) per frame
        self._queue: "queue.Queue[Optional[Frame]]" = queue.Queue(
            maxsize=dataset.fps * 4)
        self._closing  = False
        self._enqueued = 0
        self._dropped  = 0
        self._start    = time.time()
        self._thread   = threading.Thread(target=self._consume, daemon=True,
                                          name="rec-writer")
        self._thread.start()

    # producer side — called from the capture thread ────────────────────────────
    def add(self, frame: Frame) -> None:
        if self._closing:
            return
        try:
            self._queue.put_nowait(frame)
            self._enqueued += 1
        except queue.Full:
            self._dropped += 1

    @property
    def count(self) -> int:
        return self._enqueued

    # consumer side — the writer thread ─────────────────────────────────────────
    def _consume(self) -> None:
        while True:
            frame = self._queue.get()
            if frame is None:
                return
            for w, img in zip(self._writers, frame.images):
                w.write(img)
            self._meta.append((frame.timestamp, frame.action))

    def _drain(self) -> None:
        self._closing = True
        self._queue.put(None)          # sentinel: stop the writer thread
        self._thread.join()
        for w in self._writers:
            w.close()                  # flush + finalize each mp4

    # lifecycle ──────────────────────────────────────────────────────────────────
    def finalize(self) -> None:
        """Drain the encoder, then write parquet + move videos into the dataset."""
        self._drain()
        self._ds._commit_episode(self._meta, self._tmp,
                                 time.time() - self._start, self._dropped)
        shutil.rmtree(self._tmp, ignore_errors=True)

    def discard(self) -> None:
        self._drain()
        shutil.rmtree(self._tmp, ignore_errors=True)
        print("[rec] Discarded episode")


# ── LeRobot v3 dataset writer ──────────────────────────────────────────────────

class LeRobotDatasetWriter:
    """
    Writes a LeRobot-compatible dataset (codebase_version v2.1) incrementally.

    Layout:
        <root>/
          meta/info.json  episodes.jsonl  tasks.jsonl
          data/chunk-000/episode_000000.parquet
          videos/chunk-000/<cam_key>/episode_000000.mp4
    """

    def __init__(self, root: Path, cam_keys: list[str], fps: int,
                 width: int, height: int, has_trigger: bool = True,
                 task: str = DEFAULT_TASK,
                 chunks_size: int = CHUNKS_SIZE) -> None:
        self.root        = root
        self.cam_keys    = cam_keys
        self.fps         = fps
        self.width       = width
        self.height      = height
        self.has_trigger = has_trigger
        self.task        = task
        self.chunks_size = chunks_size

        self.total_episodes = 0
        self.total_frames   = 0
        self._episodes_meta: list[dict] = []
        self._lock          = threading.Lock()   # serializes commit + uid alloc
        self._next_uid      = 0

        for sub in ("meta", "data", "videos"):
            (root / sub).mkdir(parents=True, exist_ok=True)

        tasks_path = root / "meta" / "tasks.jsonl"
        if not tasks_path.exists():
            with open(tasks_path, "w") as f:
                json.dump({"task_index": 0, "task": task}, f)
                f.write("\n")

    def begin_episode(self) -> "_StreamingEpisode":
        """Open a new streaming episode; feed it frames, then finalize/discard."""
        with self._lock:
            uid = self._next_uid
            self._next_uid += 1
        return _StreamingEpisode(self, uid)

    def _commit_episode(self, meta: list[tuple[float, float]], tmp_dir: Path,
                        duration: float, dropped: int) -> None:
        """Write the parquet and move the streamed videos into the dataset.

        Called from the episode's finalize thread. The video is already encoded;
        this only writes metadata and renames files, so the lock is held briefly.
        """
        if not meta:
            print("[dataset] Episode discarded (no frames)")
            return

        with self._lock:
            ep_idx = self.total_episodes
            chunk  = ep_idx // self.chunks_size
            n      = len(meta)
            t0     = meta[0][0]
            base   = self.total_frames

            # ── Parquet ─────────────────────────────────────────────────────────
            parquet_dir = self.root / "data" / f"chunk-{chunk:03d}"
            parquet_dir.mkdir(parents=True, exist_ok=True)

            cols: dict = {
                "timestamp":     pa.array([ts - t0 for ts, _ in meta], pa.float32()),
                "frame_index":   pa.array(range(n),                    pa.int64()),
                "episode_index": pa.array([ep_idx] * n,                pa.int64()),
                "index":         pa.array(range(base, base + n),       pa.int64()),
                "task_index":    pa.array([0] * n,                     pa.int64()),
                "next.done":     pa.array([False] * (n - 1) + [True],  pa.bool_()),
            }
            if self.has_trigger:
                cols["action"] = pa.array(
                    [[a] for _, a in meta],
                    pa.list_(pa.float32()),
                )

            pq.write_table(pa.table(cols),
                           parquet_dir / f"episode_{ep_idx:06d}.parquet")

            # ── Videos: move the streamed temp files into place ──────────────────
            for cam_idx, cam_key in enumerate(self.cam_keys):
                vid_dir = self.root / "videos" / f"chunk-{chunk:03d}" / cam_key
                vid_dir.mkdir(parents=True, exist_ok=True)
                (tmp_dir / f"cam_{cam_idx}.mp4").replace(
                    vid_dir / f"episode_{ep_idx:06d}.mp4")

            # ── Metadata ─────────────────────────────────────────────────────────
            self._episodes_meta.append({
                "episode_index": ep_idx,
                "tasks":  [self.task],
                "length": n,
            })
            self.total_frames   += n
            self.total_episodes += 1
            self._flush_meta()

        warn = f"  [!] {dropped} frames DROPPED (encoder too slow)" if dropped else ""
        print(f"[dataset] Saved episode {ep_idx}  ({n} frames  {duration:.1f}s){warn}")

    def _flush_meta(self) -> None:
        ep_path = self.root / "meta" / "episodes.jsonl"
        with open(ep_path, "w") as f:
            for ep in self._episodes_meta:
                json.dump(ep, f)
                f.write("\n")

        features: dict = {
            "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
            "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
            "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
            "index":         {"dtype": "int64",   "shape": [1], "names": None},
            "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
            "next.done":     {"dtype": "bool",    "shape": [1], "names": None},
        }
        if self.has_trigger:
            features["action"] = {
                "dtype": "float32",
                "shape": [1],
                "names": ["gripper"],
            }
        for key in self.cam_keys:
            features[key] = {
                "dtype": "video",
                "shape": [self.height, self.width, 3],
                "names": ["height", "width", "channel"],
                "video_info": {
                    "video.fps":            float(self.fps),
                    "video.codec":          "h264",
                    "video.pix_fmt":        "yuv420p",
                    "video.is_depth_image": False,
                    "has_audio":            False,
                },
            }

        n_chunks = max(1, (self.total_episodes + self.chunks_size - 1)
                       // self.chunks_size) if self.total_episodes else 1
        info = {
            "codebase_version": "v2.1",
            "robot_type":       "unknown",
            "total_episodes":   self.total_episodes,
            "total_frames":     self.total_frames,
            "total_tasks":      1,
            "total_videos":     self.total_episodes * len(self.cam_keys),
            "total_chunks":     n_chunks,
            "chunks_size":      self.chunks_size,
            "fps":              self.fps,
            "splits":           {"train": f"0:{self.total_episodes}"},
            "data_path":  "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features":   features,
        }
        with open(self.root / "meta" / "info.json", "w") as f:
            json.dump(info, f, indent=2)


# ── Display helpers ────────────────────────────────────────────────────────────

_GREEN  = (0, 220, 0)
_RED    = (0, 0, 220)
_YELLOW = (0, 220, 220)
_WHITE  = (255, 255, 255)
_FONT   = cv2.FONT_HERSHEY_SIMPLEX


def _annotate(img: np.ndarray, label: str, recording: bool) -> np.ndarray:
    out   = img.copy()
    color = _GREEN if recording else _RED
    cv2.putText(out, label, (8, 26), _FONT, 0.7, color, 2)
    return out


def _status_bar(width: int, recording: bool, n_frames: int,
                n_saved: int, action: float) -> np.ndarray:
    bar = np.zeros((36, width, 3), dtype=np.uint8)
    if recording:
        msg = f"REC  {n_frames} frames  grip={action:.2f}"
        cv2.putText(bar, msg, (8, 24), _FONT, 0.7, _GREEN, 2)
    else:
        msg = f"IDLE  saved: {n_saved}  grip={action:.2f}"
        cv2.putText(bar, msg, (8, 24), _FONT, 0.7, _WHITE, 1)
    hint = "[R] rec  [S] stop  [D] discard  [Q] quit"
    cv2.putText(bar, hint, (width - 370, 24), _FONT, 0.5, _YELLOW, 1)
    return bar


# ── GUI / headless helpers ─────────────────────────────────────────────────────

def _check_display() -> bool:
    try:
        cv2.imshow("__probe__", np.zeros((1, 1, 3), dtype=np.uint8))
        cv2.waitKey(1)
        cv2.destroyWindow("__probe__")
        return True
    except cv2.error:
        return False


def _stdin_key() -> Optional[int]:
    import select, sys, tty, termios
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], 0)[0]:
            return ord(sys.stdin.read(1))
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None


# ── Main loop ──────────────────────────────────────────────────────────────────

def run(
    cam_ids: list[int],
    fps: int,
    width: int,
    height: int,
    output: Path,
    task: str,
    trigger_thread: Optional[TriggerGripperThread],
    view_width: int = 0,
) -> None:
    cam_keys    = [f"observation.images.cam_{i}" for i in range(len(cam_ids))]
    has_trigger = trigger_thread is not None

    capture = CameraCapture(cam_ids, fps, width, height, trigger_thread)
    # Use the resolution the cameras actually deliver (may differ from requested).
    width, height = capture.width, capture.height
    dataset = LeRobotDatasetWriter(output, cam_keys, fps, width, height,
                                   has_trigger=has_trigger, task=task)

    if trigger_thread:
        trigger_thread.start()
    capture.start()

    has_display = _check_display()
    print(f"Cameras {cam_ids} opened at {width}x{height} (side-by-side: "
          f"{width // 2}x{height} per eye) @ {fps} fps")
    print(f"Trigger {'enabled' if has_trigger else 'disabled (--no-trigger)'}")
    print(f"Output  → {output.resolve()}")
    print(f"Display → {'window' if has_display else 'headless (keyboard via terminal)'}")
    print("Controls: [R] record  [S] stop/save  [D] discard  [Q] quit")

    episode:        Optional[_StreamingEpisode] = None
    saving_threads: list[threading.Thread] = []

    def _handle_key(key: int) -> bool:
        nonlocal episode
        if key == ord('q'):
            return True
        elif key == ord('r'):
            if episode is None:
                episode = dataset.begin_episode()
                capture.set_recorder(episode)
                print(f"[rec] Started episode {dataset.total_episodes}")
            else:
                print("[rec] Already recording — press S to stop first")
        elif key == ord('s'):
            if episode is not None:
                buf     = episode
                episode = None
                capture.set_recorder(None)
                print(f"[rec] Saving {buf.count} frames in background…")
                t = threading.Thread(target=buf.finalize, daemon=False)
                t.start()
                saving_threads.append(t)
            else:
                print("[rec] Not recording")
        elif key == ord('d'):
            if episode is not None:
                buf     = episode
                episode = None
                capture.set_recorder(None)
                buf.discard()
            else:
                print("[rec] Not recording")
        return False

    try:
        while True:
            frame = capture.latest()
            if frame is None:
                time.sleep(0.005)
                continue

            # Recording is fed by the capture thread (see CameraCapture._run);
            # this loop only drives the preview + keyboard.
            if has_display:
                rec       = episode is not None
                annotated = [
                    _annotate(img, key.split(".")[-1], rec)
                    for img, key in zip(frame.images, cam_keys)
                ]
                combined   = np.hstack(annotated)
                status_bar = _status_bar(
                    combined.shape[1], rec,
                    episode.count if rec else 0,
                    dataset.total_episodes,
                    frame.action,
                )
                display = np.vstack([combined, status_bar])
                # Recorded frames are full-res; only the live preview is scaled
                # down so the wide stereo image fits on screen.
                if view_width and display.shape[1] > view_width:
                    scale  = view_width / display.shape[1]
                    display = cv2.resize(
                        display, (view_width, int(display.shape[0] * scale)),
                        interpolation=cv2.INTER_AREA)
                cv2.imshow("Stereo Cameras", display)
                raw_key = cv2.waitKey(1) & 0xFF
                if raw_key != 255 and _handle_key(raw_key):
                    break
            else:
                raw_key = _stdin_key()
                if raw_key is not None and _handle_key(raw_key):
                    break
                if episode is not None and episode.count % (fps * 5) == 1:
                    print(f"[rec] {episode.count} frames  grip={frame.action:.2f}")
                time.sleep(1.0 / fps)

    finally:
        if episode is not None:
            capture.set_recorder(None)
            print(f"[rec] Auto-saving open episode ({episode.count} frames)…")
            episode.finalize()

        for t in saving_threads:
            t.join()

        capture.stop()
        if trigger_thread:
            trigger_thread.stop()
        if has_display:
            cv2.destroyAllWindows()

        print(
            f"\n[done] Dataset → {output.resolve()}\n"
            f"       {dataset.total_episodes} episodes  "
            f"{dataset.total_frames} frames"
        )


# ── CLI ────────────────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description="Record stereo cameras + trigger to LeRobot dataset v3."
    )

    # Camera
    p.add_argument("--cam-ids", type=int, nargs=2, default=None,
                   metavar=("LEFT", "RIGHT"),
                   help="Camera capture indices. Omit to auto-detect the "
                        "capture nodes (skips metadata nodes).")
    p.add_argument("--fps",    type=int,  default=DEFAULT_FPS)
    p.add_argument("--width",  type=int,  default=DEFAULT_WIDTH)
    p.add_argument("--height", type=int,  default=DEFAULT_HEIGHT)
    p.add_argument("--view-width", type=int, default=1280,
                   help="Scale the live preview down to this width so it fits "
                        "the screen (recording stays full-res). 0 disables.")
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--task",   type=str,  default=DEFAULT_TASK)

    # Trigger
    p.add_argument("--no-trigger", action="store_true",
                   help="Skip trigger hardware (camera-only recording)")
    p.add_argument("--reader", choices=["feetech", "serial"], default="feetech")
    p.add_argument("--leader-port", default="/dev/ttyACM0")
    p.add_argument("--trigger-id", type=int, default=6)
    p.add_argument("--raw-min", type=float)
    p.add_argument("--raw-max", type=float)
    p.add_argument("--invert",  action="store_true")
    p.add_argument("--rate",    type=float, default=50.0,
                   help="Trigger mirror loop rate Hz (default: 50)")
    p.add_argument("--deadband", type=float, default=0.01)
    p.add_argument("--ema",      type=float, default=0.3)

    # Gripper
    p.add_argument("--gripper", choices=["feetech", "dm4310"], default="dm4310")
    p.add_argument("--follower-port", default="/dev/ttyACM1")
    p.add_argument("--gripper-id",    type=int, default=6)
    p.add_argument("--can-port",      default="/dev/ttyACM2")
    p.add_argument("--can-baud",      type=int, default=921600)
    p.add_argument("--can-id",        type=lambda x: int(x, 0), default=0x01)
    p.add_argument("--master-id",     type=lambda x: int(x, 0), default=0x11)
    p.add_argument("--gripper-open",   type=float, default=1.047)
    p.add_argument("--gripper-closed", type=float, default=0.0)
    p.add_argument("--gripper-kp",      type=float, default=1.0)
    p.add_argument("--gripper-kd",      type=float, default=1.5)
    p.add_argument("--gripper-max-vel", type=float, default=0.5)
    p.add_argument("--dm-mit-rate",     type=float)

    args = p.parse_args()

    # ── Resolve cameras ────────────────────────────────────────────────────────
    if args.cam_ids is None:
        found = find_capture_cameras()
        if len(found) < 2:
            p.error(f"auto-detected capture cameras {found}, need 2. "
                    f"Check `v4l2-ctl --list-devices` and pass --cam-ids explicitly.")
        args.cam_ids = found[:2]
        print(f"[auto] detected capture cameras: {found} -> using {args.cam_ids}")

    # ── Build trigger thread (unless --no-trigger) ─────────────────────────────
    trigger_thread: Optional[TriggerGripperThread] = None

    if not args.no_trigger:
        if args.raw_min is None or args.raw_max is None:
            p.error("--raw-min and --raw-max are required unless --no-trigger is set. "
                    "Run teleop_trigger.py --calibrate first.")

        if args.reader == "feetech":
            reader: TriggerReader = FeetechTriggerReader(
                port=args.leader_port, motor_id=args.trigger_id)
        else:
            reader = SerialTriggerReader(port=args.leader_port)

        if args.gripper == "feetech":
            gripper: GripperController = FeetechGripperController(
                port=args.follower_port, motor_id=args.gripper_id)
        else:
            gripper = DM4310GripperController(
                port=args.can_port, baud=args.can_baud,
                can_id=args.can_id, master_id=args.master_id,
                open_pos=args.gripper_open, closed_pos=args.gripper_closed,
                kp=args.gripper_kp, kd=args.gripper_kd,
                max_vel=args.gripper_max_vel, rate_hz=args.rate,
                mit_rate_hz=args.dm_mit_rate,
            )

        cfg = MirrorConfig(
            raw_min=args.raw_min, raw_max=args.raw_max,
            invert=args.invert, rate_hz=args.rate,
            deadband=args.deadband, ema_alpha=args.ema,
        )
        trigger_thread = TriggerGripperThread(reader, gripper, cfg)

    run(
        cam_ids=args.cam_ids,
        fps=args.fps,
        width=args.width,
        height=args.height,
        output=args.output,
        task=args.task,
        trigger_thread=trigger_thread,
        view_width=args.view_width,
    )


if __name__ == "__main__":
    main()
