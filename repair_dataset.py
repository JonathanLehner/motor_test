#!/usr/bin/env python3
"""
repair_dataset.py — Make a malformed LeRobot v3.0 dataset trainable.

Why
---
The recordings here have three problems that stop LeRobot (and therefore
SmolVLA) from loading/training on them:

  1. fps mismatch + jitter: meta says 30fps, but data is logged at ~20Hz with
     wall-clock (jittery) timestamps. LeRobot requires every timestamp to be
     exactly 1/fps apart (±1e-4s) — wall-clock timestamps fail this immediately.
  2. data/video misalignment: the data has ~2x as many rows as the video has
     frames, and the video's real duration (e.g. 66.5s) doesn't match the data's
     real elapsed time (e.g. 199s). LeRobot needs one video frame per data row,
     at the same timestamp.
  3. `action` dtype: declared shape [1] (a scalar) but stored as list<float>.
     LeRobot maps shape-[1] features to a scalar `Value`, so the cast fails.

What it does
------------
Resamples every episode onto a single, uniform `--fps` grid, using the data's
*real* timestamps (max timestamp = true elapsed time) as the master clock:

  - timestamps become exactly i/fps (LeRobot-compliant grid),
  - each camera video is time-stretched to the real elapsed time and re-encoded
    at `fps` (CFR), so there is exactly one video frame per data row, aligned,
  - `action` (and any other feature) is linearly resampled onto the grid and
    written with the dtype LeRobot expects (scalar for shape [1]).

The result is a fresh, valid v3.0 dataset that loads with `LeRobotDataset`.

Usage
-----
    python repair_dataset.py --input ./lerobot_dataset --output ./lerobot_clean
    python repair_dataset.py --input ./lerobot_dataset --output ./lerobot_clean --fps 20

`--fps` is your choice of unified rate (default 20 ≈ the real control rate):
  - 20: keeps all control resolution; video frames are held/duplicated to match.
  - 10: matches the camera's true ~10fps content (no duplicated video frames),
        but halves the action samples.
The original dataset is never modified.

Requirements: pandas, pyarrow, numpy, ffmpeg/ffprobe on PATH.
"""

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

META_COLS = {"timestamp", "frame_index", "episode_index", "index", "task_index", "next.done"}


# --------------------------------------------------------------------------
# Read the source v3 dataset
# --------------------------------------------------------------------------

def load_source(dataset: Path):
    info = json.loads((dataset / "meta" / "info.json").read_text())
    fps = float(info.get("fps", 30))
    cameras = sorted(k for k, v in info["features"].items()
                     if isinstance(v, dict) and v.get("dtype") == "video")
    data_tmpl = info["data_path"]
    video_tmpl = info["video_path"]

    ep_files = sorted((dataset / "meta" / "episodes").rglob("file-*.parquet"))
    edf = pd.concat([pd.read_parquet(f) for f in ep_files], ignore_index=True)
    edf = edf.sort_values("episode_index").reset_index(drop=True)

    # full metadata timespan per shared video file (to scale metadata-time -> real)
    file_span, file_cnt = {}, {}
    for _, r in edf.iterrows():
        for cam in cameras:
            key = (cam, int(r[f"videos/{cam}/chunk_index"]), int(r[f"videos/{cam}/file_index"]))
            file_span[key] = max(file_span.get(key, 0.0), float(r[f"videos/{cam}/to_timestamp"]))
            file_cnt[key] = file_cnt.get(key, 0) + 1

    episodes = []
    for _, r in edf.iterrows():
        ep = int(r["episode_index"])
        cams = {}
        for cam in cameras:
            ci, fi = int(r[f"videos/{cam}/chunk_index"]), int(r[f"videos/{cam}/file_index"])
            cams[cam] = {
                "path": dataset / video_tmpl.format(video_key=cam, chunk_index=ci, file_index=fi),
                "from": float(r[f"videos/{cam}/from_timestamp"]),
                "to": float(r[f"videos/{cam}/to_timestamp"]),
                "fspan": file_span[(cam, ci, fi)],
                "fcount": file_cnt[(cam, ci, fi)],
            }
        episodes.append({
            "index": ep,
            "data_path": dataset / data_tmpl.format(
                chunk_index=int(r["data/chunk_index"]), file_index=int(r["data/file_index"])),
            "cams": cams,
        })
    return info, fps, cameras, episodes


def data_feature_specs(info: dict, cameras: list):
    """Non-video, non-meta features we must resample, with their (shape, dtype)."""
    specs = {}
    for key, ft in info["features"].items():
        if key in cameras or key in META_COLS:
            continue
        if ft.get("dtype") == "video":
            continue
        specs[key] = (tuple(ft.get("shape", [1])), ft.get("dtype", "float32"))
    return specs


# --------------------------------------------------------------------------
# ffmpeg / ffprobe helpers
# --------------------------------------------------------------------------

def ffprobe_duration(path: Path) -> float:
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration",
                          "-of", "default=nw=1:nk=1", str(path)],
                         check=True, capture_output=True, text=True).stdout.strip()
    return float(out) if out else 0.0


def ffprobe_nbframes(path: Path) -> int:
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-count_frames",
                          "-show_entries", "stream=nb_read_frames", "-of", "default=nw=1:nk=1",
                          str(path)], check=True, capture_output=True, text=True).stdout.strip()
    return int(out) if out.isdigit() else 0


def encode_stretched(src: Path, seg_from: float, seg_to: float, stretch: float,
                     fps: float, dst: Path):
    """Cut [seg_from, seg_to] (real seconds) of src, stretch its PTS by `stretch`
    (to real elapsed time), and re-encode CFR at `fps`. seg_to<=seg_from means
    'use the whole input'. Output is self-contained."""
    vf = f"setpts={stretch:.6f}*PTS,fps={fps:g}"
    # Cut on the INPUT (before -i) so the segment is selected *before* setpts
    # stretches the timeline; otherwise -to would trim the stretched output.
    cmd = ["ffmpeg", "-y", "-loglevel", "error"]
    if seg_from > 0:
        cmd += ["-ss", f"{seg_from:.6f}"]
    if seg_to > seg_from:
        cmd += ["-to", f"{seg_to:.6f}"]
    cmd += ["-i", str(src), "-vf", vf, "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(dst)]
    subprocess.run(cmd, check=True)


def detect_vstack(src: Path) -> bool:
    """True if the video's frames are two captures stacked top/bottom (the
    recorder mis-framed 640x240 captures as 640x480). Heuristic: across sampled
    frames the top half is much more similar to the bottom half than the left
    half is to the right half."""
    cap = cv2.VideoCapture(str(src))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    idxs = [int(n * f) for f in (0.2, 0.4, 0.6, 0.8)] if n > 5 else [0]
    tb, lr, got = 0.0, 0.0, 0
    for i in idxs:
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, fr = cap.read()
        if not ok or fr.shape[0] % 2 or fr.shape[1] % 2:
            continue
        h, w = fr.shape[:2]
        H, W = h // 2, w // 2
        tb += float(np.abs(fr[:H].astype(int) - fr[H:2 * H].astype(int)).mean())
        lr += float(np.abs(fr[:, :W].astype(int) - fr[:, W:2 * W].astype(int)).mean())
        got += 1
    cap.release()
    return got > 0 and tb < 0.6 * lr   # clearly more similar vertically -> stacked


def destack_to_temp(src: Path, real_dur: float, dst: Path) -> tuple[int, int]:
    """Split every frame into its top and bottom halves and write them as
    consecutive frames (the true capture order), recovering 2x the frames at
    half height. Returns (width, half_height). Streams to keep memory flat."""
    nb = ffprobe_nbframes(src)
    cap = cv2.VideoCapture(str(src))
    ok, fr = cap.read()
    if not ok:
        raise RuntimeError(f"cannot read {src}")
    h, w = fr.shape[:2]
    half = h // 2
    temp_fps = max(1.0, (2 * nb) / real_dur) if real_dur > 0 else 20.0  # -> temp duration ~= real
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo",
         "-s", f"{w}x{half}", "-pix_fmt", "bgr24", "-r", f"{temp_fps:.6f}", "-i", "pipe:0",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(dst)],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while True:
        proc.stdin.write(np.ascontiguousarray(fr[:half]).tobytes())
        proc.stdin.write(np.ascontiguousarray(fr[half:2 * half]).tobytes())
        ok, fr = cap.read()
        if not ok:
            break
    cap.release()
    proc.stdin.close()
    proc.wait()
    return w, half


# --------------------------------------------------------------------------
# Repair
# --------------------------------------------------------------------------

def repair(input_dir: Path, output_dir: Path, target_fps: float):
    for tool in ("ffmpeg", "ffprobe"):
        if shutil.which(tool) is None:
            raise SystemExit(f"[error] {tool} not found on PATH.")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"[error] output dir {output_dir} is not empty — choose a fresh path.")

    info, src_fps, cameras, episodes = load_source(input_dir)
    specs = data_feature_specs(info, cameras)
    print(f"[repair] {len(episodes)} episode(s), cameras={cameras}, "
          f"features={list(specs)}, target fps={target_fps:g}")

    (output_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (output_dir / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)

    new_rows, global_index = [], 0
    out_dims = {}   # cam -> (width, height) of the produced video (for info.json shape)

    for new_idx, ep in enumerate(episodes):
        df = pd.read_parquet(ep["data_path"])
        df = df[df["episode_index"] == ep["index"]].sort_values("frame_index").reset_index(drop=True)
        src_ts = df["timestamp"].to_numpy(dtype=np.float64)
        src_ts = src_ts - src_ts[0]
        real_dur = float(src_ts[-1]) if len(src_ts) > 1 else len(df) / src_fps
        if real_dur <= 0:
            print(f"  [skip] ep {ep['index']}: zero duration")
            continue

        # --- re-encode each camera, stretched to real time, CFR at target_fps ---
        n_out = None
        with tempfile.TemporaryDirectory() as td:
            for cam in cameras:
                c = ep["cams"][cam]
                dst_dir = output_dir / "videos" / cam / "chunk-000"
                dst_dir.mkdir(parents=True, exist_ok=True)
                dst = dst_dir / f"file-{new_idx:03d}.mp4"

                if detect_vstack(c["path"]):
                    # Frames are two stacked captures: de-stack to the true ~2x
                    # frames at half height, then resample that to target_fps.
                    tmp = Path(td) / f"{cam.replace('/', '_')}_{new_idx}.mp4"
                    w, half = destack_to_temp(c["path"], real_dur, tmp)
                    stretch = real_dur / ffprobe_duration(tmp) if ffprobe_duration(tmp) > 0 else 1.0
                    encode_stretched(tmp, 0.0, 0.0, stretch, target_fps, dst)
                    out_dims[cam] = (w, half)
                else:
                    vid_real = ffprobe_duration(c["path"])
                    scale = (vid_real / c["fspan"]) if c["fspan"] > 0 else 1.0
                    seg_from, seg_to = c["from"] * scale, c["to"] * scale
                    seg_dur = (seg_to - seg_from) if seg_to > seg_from else vid_real
                    stretch = real_dur / seg_dur if seg_dur > 0 else 1.0
                    encode_stretched(c["path"], seg_from, seg_to, stretch, target_fps, dst)

                nb = ffprobe_nbframes(dst)
                n_out = nb if n_out is None else min(n_out, nb)
        if not n_out:
            print(f"  [skip] ep {ep['index']}: no video frames produced")
            continue

        # --- build aligned data on the i/fps grid (one row per video frame) ---
        t_grid = np.arange(n_out) / target_fps
        out = {
            "timestamp": pd.array(t_grid.astype(np.float32)),
            "frame_index": pd.array(np.arange(n_out, dtype=np.int64)),
            "episode_index": pd.array(np.full(n_out, new_idx, dtype=np.int64)),
            "index": pd.array(np.arange(global_index, global_index + n_out, dtype=np.int64)),
            "task_index": pd.array(np.zeros(n_out, dtype=np.int64)),
            "next.done": pd.array(np.array([False] * (n_out - 1) + [True])),
        }
        if "task_index" in df:
            out["task_index"] = pd.array(
                np.round(np.interp(t_grid, src_ts, df["task_index"].to_numpy(dtype=np.float64)))
                .astype(np.int64))

        # resample each data feature onto the grid (linear interp per component)
        for key, (shape, dtype) in specs.items():
            col = df[key].to_numpy()
            arr = np.array([np.ravel(v) for v in col], dtype=np.float64)  # (N_src, prod(shape))
            res = np.stack([np.interp(t_grid, src_ts, arr[:, j]) for j in range(arr.shape[1])], axis=1)
            if shape == (1,):                       # LeRobot: scalar Value
                out[key] = pd.array(res[:, 0].astype(dtype))
            else:                                   # LeRobot: Sequence(list)
                out[key] = list(res.astype(dtype))

        pd.DataFrame(out).to_parquet(
            output_dir / "data" / "chunk-000" / f"file-{new_idx:03d}.parquet", index=False)

        dur = n_out / target_fps
        row = {"episode_index": new_idx, "tasks": ["teleop recording"], "length": n_out,
               "dataset_from_index": global_index, "dataset_to_index": global_index + n_out,
               "data/chunk_index": 0, "data/file_index": new_idx}
        for cam in cameras:
            row[f"videos/{cam}/chunk_index"] = 0
            row[f"videos/{cam}/file_index"] = new_idx
            row[f"videos/{cam}/from_timestamp"] = 0.0
            row[f"videos/{cam}/to_timestamp"] = dur
        new_rows.append(row)
        print(f"  ep {ep['index']:>3d}: {len(df)} rows @~{len(df)/real_dur:.0f}Hz, {real_dur:.1f}s "
              f"-> {n_out} frames @ {target_fps:g}fps  (sub-ep {new_idx:03d})")
        global_index += n_out

    if not new_rows:
        raise SystemExit("[error] nothing repaired.")

    pd.DataFrame(new_rows).to_parquet(
        output_dir / "meta" / "episodes" / "chunk-000" / "file-000.parquet", index=False)

    # tasks.parquet: carry over if present, else build a minimal one
    src_tasks = input_dir / "meta" / "tasks.parquet"
    if src_tasks.exists():
        shutil.copy2(src_tasks, output_dir / "meta" / "tasks.parquet")
    else:
        pd.DataFrame({"task_index": [0]},
                     index=pd.Index(["teleop recording"], name="task")
                     ).to_parquet(output_dir / "meta" / "tasks.parquet")

    # info.json: same features, new fps + totals; action shape [1] now matches scalar storage
    new_info = dict(info)
    new_info["fps"] = int(target_fps) if float(target_fps).is_integer() else target_fps
    new_info["total_episodes"] = len(new_rows)
    new_info["total_frames"] = global_index
    new_info["total_tasks"] = 1
    new_info["splits"] = {"train": f"0:{len(new_rows)}"}
    for cam in cameras:
        new_info["features"][cam].setdefault("info", {})["video.fps"] = float(target_fps)
        if cam in out_dims:                      # de-stacked -> true (half-height) resolution
            w, h = out_dims[cam]
            new_info["features"][cam]["shape"] = [h, w, 3]
    (output_dir / "meta" / "info.json").write_text(json.dumps(new_info, indent=2))
    (output_dir / "meta" / "stats.json").write_text(json.dumps({}, indent=2))

    print(f"\n[repair] done — {len(new_rows)} episodes, {global_index} frames @ {target_fps:g}fps")
    print(f"[repair] -> {output_dir}")
    print("[verify] python -c \"from lerobot.datasets.lerobot_dataset import LeRobotDataset; "
          f"d=LeRobotDataset('x', root='{output_dir}'); print(d); print(d[0].keys())\"")


def main():
    ap = argparse.ArgumentParser(description="Repair a malformed LeRobot v3.0 dataset for training.")
    ap.add_argument("--input", required=True, type=Path, help="Source dataset root.")
    ap.add_argument("--output", required=True, type=Path, help="Destination for the repaired dataset.")
    ap.add_argument("--fps", type=float, default=20.0, help="Unified target fps (default 20).")
    args = ap.parse_args()
    inp = args.input.expanduser().resolve()
    if not (inp / "meta" / "info.json").exists():
        raise SystemExit(f"[error] {inp} is not a v3.0 dataset (no meta/info.json).")
    repair(inp, args.output.expanduser().resolve(), args.fps)


if __name__ == "__main__":
    main()
