#!/usr/bin/env python3
"""
episode_picker.py — Split LeRobot v3.0 episodes into sub-episodes (clips).

What it does
------------
- Reads a LeRobot **v3.0** dataset, finds every episode and every camera.
- Serves a local web page where you watch each episode and carve it into
  sub-episodes by marking in/out clips (like a video editor).
- Saves your clips to episode_clips.json (survives restarts — stop and resume).
- Exports a clean v3.0 dataset where every clip becomes its own episode.
  Anything you did NOT mark as a clip is dropped.

Mental model
------------
One long teleop recording often contains several distinct attempts (e.g. several
cylinder pick-ups) plus dead time in between. You watch the episode, mark an
in-point and an out-point around each good attempt, and each marked clip is
exported as a standalone sub-episode. The dead time between clips is discarded.

Clips are stored as fractions [start, end] of the parent episode (0..1). On
export, fractions map to a data frame range (parquet slice) and to a video time
range (ffmpeg cut), so the data and video of each sub-episode stay aligned even
though this dataset logs data faster than the video fps.

LeRobot v3.0 layout (what this script expects)
----------------------------------------------
    meta/info.json
        codebase_version: "v3.0"
        data_path  : data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet
        video_path : videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4
        features.<key>.dtype == "video"   -> that key is a camera
    meta/episodes/chunk-XXX/file-XXX.parquet
        one row per episode: episode_index, length, tasks,
        dataset_from_index, dataset_to_index, data/chunk_index, data/file_index,
        videos/<cam>/{chunk_index,file_index,from_timestamp,to_timestamp}
    data/chunk-XXX/file-XXX.parquet     (frames for possibly several episodes)
    videos/<cam>/chunk-XXX/file-XXX.mp4 (frames for possibly several episodes)

Usage
-----
    python episode_picker.py --dataset /path/to/lerobot_dataset
    # open the printed URL, mark clips, then:
    python episode_picker.py --dataset /path/to/lerobot_dataset --export /path/to/clips_dataset

Requirements
------------
- pandas + pyarrow (to read/write parquet metadata — mandatory for v3).
- ffmpeg on PATH (only for --export, to cut clips out of videos).
- Clips live in episode_clips.json inside the dataset folder. The original
  dataset is never modified; export writes a separate copy.
"""

import argparse
import http.server
import json
import re
import shutil
import socketserver
import subprocess
import urllib.parse
from pathlib import Path

try:
    import pandas as pd
except ImportError:
    raise SystemExit(
        "[error] pandas (with pyarrow) is required to read LeRobot v3 metadata.\n"
        "        pip install pandas pyarrow"
    )


# --------------------------------------------------------------------------
# Dataset scanning (v3.0, metadata-driven)
# --------------------------------------------------------------------------

def load_info(dataset: Path) -> dict:
    info_path = dataset / "meta" / "info.json"
    if not info_path.exists():
        raise SystemExit(
            f"[error] {info_path} not found.\n"
            f"        Point --dataset at the dataset root (the folder with meta/, data/, videos/)."
        )
    info = json.loads(info_path.read_text())
    ver = str(info.get("codebase_version", "?"))
    if not ver.startswith("v3"):
        print(f"[warn] codebase_version is {ver!r}, not v3.x — this tool expects v3.0 layout.")
    return info


def camera_keys(info: dict) -> list:
    """Camera = any feature whose dtype is 'video'."""
    cams = [k for k, v in info.get("features", {}).items()
            if isinstance(v, dict) and v.get("dtype") == "video"]
    return sorted(cams)


def read_episodes_meta(dataset: Path):
    """Read every meta/episodes/**/file-*.parquet -> one DataFrame, one row per episode."""
    ep_dir = dataset / "meta" / "episodes"
    if not ep_dir.is_dir():
        raise SystemExit(f"[error] {ep_dir} not found — is this really a v3.0 dataset?")
    files = sorted(ep_dir.rglob("file-*.parquet"))
    if not files:
        raise SystemExit(f"[error] no episode metadata parquet under {ep_dir}")
    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    return df.sort_values("episode_index").reset_index(drop=True)


def scan_dataset(dataset: Path):
    """
    Returns a dict describing the dataset:
      info, fps, cameras, episodes (sorted indices), video_tmpl, data_tmpl,
      meta : ep_index -> {length, tasks, data_chunk, data_file,
                          cams: {cam -> {path, from, to, chunk, file, fspan, fcount}}}
    fspan  = full metadata timespan of the shared video file (max to_timestamp).
    fcount = how many episodes share that video file.
    """
    info = load_info(dataset)
    fps = float(info.get("fps", 30))
    cameras = camera_keys(info)
    if not cameras:
        raise SystemExit("[error] no 'video' features found in meta/info.json")
    video_tmpl = info["video_path"]
    data_tmpl = info["data_path"]

    df = read_episodes_meta(dataset)

    meta = {}
    for _, r in df.iterrows():
        ep = int(r["episode_index"])
        cams = {}
        for cam in cameras:
            ci = int(r[f"videos/{cam}/chunk_index"])
            fi = int(r[f"videos/{cam}/file_index"])
            frm = float(r[f"videos/{cam}/from_timestamp"])
            to = float(r[f"videos/{cam}/to_timestamp"])
            path = dataset / video_tmpl.format(video_key=cam, chunk_index=ci, file_index=fi)
            cams[cam] = {"path": path, "from": frm, "to": to, "chunk": ci, "file": fi}
        meta[ep] = {
            "length": int(r["length"]),
            "tasks": list(r["tasks"]) if r["tasks"] is not None else [],
            "data_chunk": int(r["data/chunk_index"]),
            "data_file": int(r["data/file_index"]),
            "cams": cams,
        }

    # Real elapsed wall-time per episode = max(data timestamp). The camera video
    # is effectively a time-lapse (encoded fps > capture rate), so its playback
    # duration is shorter than the real motion. We send this so the browser can
    # slow playback to real-time. Read only the two columns we need, once per
    # data file (a file may hold several episodes).
    real_dur = {}
    seen_data = set()
    for ep, m in meta.items():
        key = (m["data_chunk"], m["data_file"])
        if key in seen_data:
            continue
        seen_data.add(key)
        dpath = dataset / data_tmpl.format(chunk_index=key[0], file_index=key[1])
        try:
            dd = pd.read_parquet(dpath, columns=["episode_index", "timestamp"])
            for e, g in dd.groupby("episode_index"):
                t = float(g["timestamp"].max())
                real_dur[int(e)] = t if t > 0 else float("nan")
        except Exception as exc:
            print(f"[warn] could not read timestamps from {dpath}: {exc}")
    for ep, m in meta.items():
        rd = real_dur.get(ep)
        m["real_dur"] = rd if (rd and rd == rd) else m["length"] / fps  # fallback: length/fps

    # Per physical video file: how many episodes share it and its full metadata
    # timespan. This dataset logs data faster than the video fps, so to_timestamp
    # does NOT equal the real clip length — callers scale the window by
    # (real_duration / fspan) to land on the right seconds.
    file_eps, file_span = {}, {}
    for ep, m in meta.items():
        for cam, c in m["cams"].items():
            key = (cam, c["chunk"], c["file"])
            file_eps.setdefault(key, []).append(ep)
            file_span[key] = max(file_span.get(key, 0.0), c["to"])
    for ep, m in meta.items():
        for cam, c in m["cams"].items():
            key = (cam, c["chunk"], c["file"])
            c["fspan"] = file_span[key]
            c["fcount"] = len(file_eps[key])

    episodes = sorted(meta.keys())
    for ep in episodes:
        for cam, c in meta[ep]["cams"].items():
            if not c["path"].exists():
                print(f"[warn] episode {ep} camera {cam}: missing video file {c['path']}")

    return {
        "info": info, "fps": fps, "cameras": cameras, "episodes": episodes,
        "meta": meta, "video_tmpl": video_tmpl, "data_tmpl": data_tmpl,
    }


# --------------------------------------------------------------------------
# Clips persistence — episode_clips.json: { "<ep>": [[start_frac, end_frac], ...] }
# --------------------------------------------------------------------------

def clips_file(dataset: Path) -> Path:
    return dataset / "episode_clips.json"


def load_clips(dataset: Path) -> dict:
    f = clips_file(dataset)
    if f.exists():
        try:
            raw = json.loads(f.read_text())
            # normalise: {str ep: [[a,b], ...]} with 0<=a<b<=1
            out = {}
            for ep, clips in raw.items():
                cleaned = []
                for c in clips:
                    a, b = float(c[0]), float(c[1])
                    a, b = max(0.0, min(a, b)), min(1.0, max(a, b))
                    if b - a > 1e-4:
                        cleaned.append([round(a, 6), round(b, 6)])
                if cleaned:
                    out[str(ep)] = sorted(cleaned)
            return out
        except Exception:
            print(f"[warn] could not parse {f}, starting fresh")
    return {}


def save_clips(dataset: Path, clips: dict):
    clips_file(dataset).write_text(json.dumps(clips, indent=2, sort_keys=True))


# --------------------------------------------------------------------------
# Export — every clip becomes its own sub-episode in a new v3.0 dataset
# --------------------------------------------------------------------------

def _ffprobe_duration(path: Path) -> float:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "default=nw=1:nk=1", str(path)],
        check=True, capture_output=True, text=True).stdout.strip()
    try:
        return float(out)
    except ValueError:
        return 0.0


def _ffmpeg_cut(src: Path, dst: Path, start: float, end: float):
    """Cut [start, end] (real video seconds) out of src, re-encoding for an
    accurate, self-contained clip (-ss/-to AFTER -i trims on output)."""
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", str(src)]
    if start and start > 0:
        cmd += ["-ss", f"{start:.6f}"]
    if end and end > start:
        cmd += ["-to", f"{end:.6f}"]
    cmd += ["-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(dst)]
    subprocess.run(cmd, check=True)


def _export_clip_video(c: dict, a: float, b: float, dst: Path, dur_cache: dict):
    """
    Write the [a,b]-fraction clip of one episode for one camera.

    The episode occupies [from,to] (metadata-time) of its source file; scale that
    to real seconds via (real_duration / fspan), then take the [a,b] slice of that
    real window. If the clip covers a whole single-episode file, copy losslessly.
    """
    src = c["path"]
    whole = (c["fcount"] == 1 and c["from"] <= 0.05 and a <= 0.001 and b >= 0.999)
    if whole:
        shutil.copy2(src, dst)
        return
    real = dur_cache.get(src)
    if real is None:
        real = dur_cache[src] = _ffprobe_duration(src)
    scale = (real / c["fspan"]) if c["fspan"] > 0 else 1.0
    ep_from, ep_to = c["from"] * scale, c["to"] * scale
    span = ep_to - ep_from
    _ffmpeg_cut(src, dst, ep_from + a * span, ep_from + b * span)


def export_clips(dataset: Path, export_dir: Path, clips: dict):
    if shutil.which("ffmpeg") is None:
        raise SystemExit("[error] ffmpeg not found on PATH — needed to cut clips.")

    ds = scan_dataset(dataset)
    info, fps, cameras, meta = ds["info"], ds["fps"], ds["cameras"], ds["meta"]
    data_tmpl = ds["data_tmpl"]

    # Flatten to an ordered list of (old_ep, a, b), episode then clip start.
    jobs = []
    for ep_str, clip_list in clips.items():
        ep = int(ep_str)
        if ep not in meta:
            print(f"[warn] clips reference episode {ep} which is not in the dataset — skipped")
            continue
        for a, b in sorted(clip_list):
            jobs.append((ep, float(a), float(b)))
    if not jobs:
        raise SystemExit("[error] no clips marked — open the picker and mark some in/out clips first.")

    if export_dir.exists() and any(export_dir.iterdir()):
        raise SystemExit(f"[error] export dir {export_dir} is not empty — choose a fresh path.")
    (export_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (export_dir / "meta" / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)

    print(f"[export] {len(jobs)} clip(s) -> sub-episodes in {export_dir}")

    new_rows, dur_cache, src_cache = [], {}, {}
    global_index, new_idx = 0, 0

    for old_ep, a, b in jobs:
        m = meta[old_ep]
        length = m["length"]
        i0, i1 = round(a * length), round(b * length)
        i0, i1 = max(0, min(i0, length)), max(0, min(i1, length))
        if i1 <= i0:
            print(f"  [skip] ep {old_ep} clip [{a:.3f},{b:.3f}] is empty")
            continue

        # --- data: slice episode rows by frame range, rebase, reindex ---
        if old_ep not in src_cache:
            src_data = dataset / data_tmpl.format(chunk_index=m["data_chunk"], file_index=m["data_file"])
            edf = pd.read_parquet(src_data)
            edf = edf[edf["episode_index"] == old_ep].sort_values("frame_index").reset_index(drop=True)
            src_cache[old_ep] = edf
        sub = src_cache[old_ep].iloc[i0:i1].copy().reset_index(drop=True)
        n = len(sub)
        sub["episode_index"] = new_idx
        sub["frame_index"] = range(n)
        sub["index"] = range(global_index, global_index + n)
        sub["timestamp"] = (sub["timestamp"] - sub["timestamp"].iloc[0]).astype("float32")
        sub.to_parquet(export_dir / "data" / "chunk-000" / f"file-{new_idx:03d}.parquet", index=False)

        # --- videos: cut the [a,b] window for each camera ---
        for cam in cameras:
            dst_dir = export_dir / "videos" / cam / "chunk-000"
            dst_dir.mkdir(parents=True, exist_ok=True)
            _export_clip_video(m["cams"][cam], a, b, dst_dir / f"file-{new_idx:03d}.mp4", dur_cache)

        # --- per-episode metadata row ---
        dur = n / fps
        row = {
            "episode_index": new_idx, "tasks": m["tasks"], "length": n,
            "dataset_from_index": global_index, "dataset_to_index": global_index + n,
            "data/chunk_index": 0, "data/file_index": new_idx,
        }
        for cam in cameras:
            row[f"videos/{cam}/chunk_index"] = 0
            row[f"videos/{cam}/file_index"] = new_idx
            row[f"videos/{cam}/from_timestamp"] = 0.0
            row[f"videos/{cam}/to_timestamp"] = dur
        new_rows.append(row)

        print(f"  ep {old_ep:>3d} [{a:.3f},{b:.3f}]  ->  sub-ep {new_idx:03d}  ({n} frames, {dur:.1f}s)")
        new_idx += 1
        global_index += n

    if not new_rows:
        raise SystemExit("[error] every clip was empty — nothing exported.")

    pd.DataFrame(new_rows).to_parquet(
        export_dir / "meta" / "episodes" / "chunk-000" / "file-000.parquet", index=False)

    for fname in ("tasks.parquet", "stats.json"):
        src = dataset / "meta" / fname
        if src.exists():
            shutil.copy2(src, export_dir / "meta" / fname)

    new_info = dict(info)
    new_info["total_episodes"] = new_idx
    new_info["total_frames"] = global_index
    new_info["splits"] = {"train": f"0:{new_idx}"}
    (export_dir / "meta" / "info.json").write_text(json.dumps(new_info, indent=2))

    print(f"\n[export] done — {new_idx} sub-episodes, {global_index} frames -> {export_dir}")
    if (dataset / "meta" / "stats.json").exists() and \
            json.loads((dataset / "meta" / "stats.json").read_text()):
        print("[note] stats.json was copied verbatim; recompute it if you rely on dataset stats.")


# --------------------------------------------------------------------------
# Web server
# --------------------------------------------------------------------------

def build_app(dataset: Path):
    ds = scan_dataset(dataset)
    info, fps, cameras = ds["info"], ds["fps"], ds["cameras"]
    episodes, meta = ds["episodes"], ds["meta"]
    clips = load_clips(dataset)

    print(f"[scan] dataset : {dataset}  (codebase {info.get('codebase_version')})")
    print(f"[scan] cameras : {cameras}")
    print(f"[scan] episodes: {len(episodes)}  (indices {episodes[0] if episodes else '-'}"
          f"..{episodes[-1] if episodes else '-'})")

    lengths = {ep: meta[ep]["length"] for ep in episodes}
    if lengths:
        med = sorted(lengths.values())[len(lengths) // 2]
        small_threshold = max(med * 0.1, 30)
        small = sorted(ep for ep, n in lengths.items() if n < small_threshold)
        if small:
            print(f"[warn] {len(small)} episode(s) look very short "
                  f"(<{small_threshold:.0f} frames) — likely empty/truncated: {small}")
    else:
        small_threshold = 0

    def video_path(cam, ep):
        rec = meta.get(ep)
        return rec["cams"].get(cam, {}).get("path") if rec else None

    state = {"clips": clips}

    # Optional AprilTag cube overlay — sidecar written by apriltag_cube_pose.py.
    cube_path = dataset / "apriltag_cube_pose.parquet"
    cube_meta_path = dataset / "apriltag_cube_pose.meta.json"
    cube_state = {"loaded": False, "meta": {}, "by_ep": {}}

    def _clean(v):
        v = float(v)
        return None if v != v else round(v, 4)

    def _corners(v):
        if v is None:
            return None
        try:
            arr = [float(x) for x in v]
        except TypeError:
            return None
        if len(arr) != 8 or any(x != x for x in arr):
            return None
        return [round(x, 2) for x in arr]

    def load_cube():
        if cube_state["loaded"]:
            return
        cube_state["loaded"] = True
        if not cube_path.exists():
            return
        try:
            cube_state["meta"] = (json.loads(cube_meta_path.read_text())
                                  if cube_meta_path.exists() else {})
            cdf = pd.read_parquet(cube_path)
        except Exception as exc:
            print(f"[warn] could not load cube overlay {cube_path}: {exc}")
            return
        has_corners = "left_corners" in cdf.columns and "right_corners" in cdf.columns
        by_ep = {}
        for r in cdf.itertuples(index=False):
            if int(r.tag_id) < 0:
                continue
            by_ep.setdefault(int(r.episode_index), []).append([
                int(r.video_frame),
                _clean(r.left_cx), _clean(r.left_cy), _clean(r.right_cx), _clean(r.right_cy),
                _clean(r.stereo_X), _clean(r.stereo_Y), _clean(r.stereo_Z),
                _clean(r.pnp_left_X), _clean(r.pnp_left_Y), _clean(r.pnp_left_Z),
                _corners(r.left_corners) if has_corners else None,
                _corners(r.right_corners) if has_corners else None,
            ])
        cube_state["by_ep"] = by_ep
        print(f"[info] cube overlay: {sum(len(v) for v in by_ep.values())} detections "
              f"across {len(by_ep)} episode(s) on {cube_state['meta'].get('camera')}")

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _counts(self):
            nclips = sum(len(v) for v in state["clips"].values())
            neps = sum(1 for v in state["clips"].values() if v)
            return {"clips": nclips, "episodes_with_clips": neps}

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)
            path = parsed.path

            if path == "/":
                self._send(200, INDEX_HTML, "text/html; charset=utf-8")
                return

            if path == "/api/episodes":
                data = []
                for ep in episodes:
                    m = meta[ep]
                    seg = {cam: [m["cams"][cam]["from"], m["cams"][cam]["to"],
                                 m["cams"][cam]["fspan"]] for cam in cameras}
                    dur = max((to - frm) for frm, to, _ in seg.values()) if seg else 0
                    data.append({
                        "index": ep, "frames": m["length"], "dur": round(dur, 1),
                        "real_dur": round(m["real_dur"], 2),
                        "tasks": m["tasks"], "seg": seg,
                        "clips": state["clips"].get(str(ep), []),
                    })
                self._send(200, json.dumps({
                    "cameras": cameras, "fps": fps, "episodes": data,
                    "small_threshold": small_threshold,
                }))
                return

            if path == "/api/cube":
                load_cube()
                q = urllib.parse.parse_qs(parsed.query)
                try:
                    ep = int(q["ep"][0])
                except (KeyError, ValueError):
                    ep = None
                m = cube_state["meta"]
                self._send(200, json.dumps({
                    "camera": m.get("camera"), "frame_w": m.get("frame_w"),
                    "frame_h": m.get("frame_h"), "eye_w": m.get("eye_w"),
                    "dets": cube_state["by_ep"].get(ep, []),
                }))
                return

            if path == "/video":
                q = urllib.parse.parse_qs(parsed.query)
                try:
                    cam, ep = q["cam"][0], int(q["ep"][0])
                except (KeyError, ValueError):
                    self._send(400, "bad request", "text/plain")
                    return
                vp = video_path(cam, ep)
                if vp is None or not vp.exists():
                    self._send(404, "no video", "text/plain")
                    return
                self._serve_file(vp)
                return

            self._send(404, "not found", "text/plain")

        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path != "/api/clips":
                self._send(404, "not found", "text/plain")
                return
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or "{}")
            ep = str(payload.get("ep"))
            new_clips = []
            for c in payload.get("clips", []):
                a, b = float(c[0]), float(c[1])
                a, b = max(0.0, min(a, b)), min(1.0, max(a, b))
                if b - a > 1e-4:
                    new_clips.append([round(a, 6), round(b, 6)])
            if new_clips:
                state["clips"][ep] = sorted(new_clips)
            else:
                state["clips"].pop(ep, None)
            save_clips(dataset, state["clips"])
            self._send(200, json.dumps({"ok": True, **self._counts()}))

        def _serve_file(self, fp: Path):
            """Serve an mp4 with HTTP Range support so the browser can scrub."""
            fsize = fp.stat().st_size
            rng = self.headers.get("Range")
            if rng:
                m = re.match(r"bytes=(\d+)-(\d*)", rng)
                start = int(m.group(1))
                end = int(m.group(2)) if m.group(2) else fsize - 1
                end = min(end, fsize - 1)
                length = end - start + 1
                self.send_response(206)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Content-Range", f"bytes {start}-{end}/{fsize}")
                self.send_header("Content-Length", str(length))
                self.end_headers()
                with open(fp, "rb") as f:
                    f.seek(start)
                    self.wfile.write(f.read(length))
            else:
                self.send_response(200)
                self.send_header("Content-Type", "video/mp4")
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Content-Length", str(fsize))
                self.end_headers()
                with open(fp, "rb") as f:
                    shutil.copyfileobj(f, self.wfile)

    return Handler


INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Episode Splitter</title>
<style>
  :root {
    --bg: #14171c; --panel: #1d2128; --line: #2c313b;
    --ink: #e8eaed; --mut: #8b93a1;
    --clip: #3fb950; --pend: #d8a657; --accent: #d8a657; --play: #58a6ff;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--ink);
    font: 15px/1.5 ui-monospace, "JetBrains Mono", Menlo, Consolas, monospace;
  }
  header {
    padding: 14px 20px; border-bottom: 1px solid var(--line);
    display: flex; align-items: baseline; gap: 18px; flex-wrap: wrap;
  }
  header h1 { font-size: 16px; margin: 0; letter-spacing: .5px; }
  header .stat { color: var(--mut); font-size: 13px; }
  header .stat b { color: var(--ink); }
  header .stat .c { color: var(--clip); }
  .wrap { display: flex; height: calc(100vh - 53px); }
  #list { width: 240px; border-right: 1px solid var(--line); overflow-y: auto; flex-shrink: 0; }
  .ep {
    padding: 9px 14px; border-bottom: 1px solid var(--line);
    cursor: pointer; display: flex; align-items: center; gap: 8px; font-size: 13px;
  }
  .ep:hover { background: var(--panel); }
  .ep.active { background: var(--panel); border-left: 3px solid var(--accent); padding-left: 11px; }
  .ep .name { flex: 1; }
  .ep .sz { color: var(--mut); font-size: 11px; }
  .ep .dot { width: 9px; height: 9px; border-radius: 50%; background: #3a3f4a; flex-shrink: 0; }
  .ep.has .dot { background: var(--clip); }
  .ep .warn { color: var(--pend); font-size: 11px; }
  #viewer { flex: 1; padding: 22px; overflow-y: auto; }
  #viewer h2 { margin: 0 0 4px; font-size: 18px; }
  #viewer .sub { color: var(--mut); font-size: 13px; margin-bottom: 16px; }
  .vids { display: flex; gap: 16px; flex-wrap: wrap; margin-bottom: 14px; }
  .vid { background: #000; border: 1px solid var(--line); border-radius: 6px; overflow: hidden; position: relative; }
  .vid video { display: block; width: 440px; max-width: 46vw; background: #000; }
  canvas.cubeov { position: absolute; left: 0; top: 0; pointer-events: none; }
  .vid .dl {
    position: absolute; top: 6px; right: 6px; z-index: 2;
    width: 26px; height: 26px; border-radius: 5px;
    display: flex; align-items: center; justify-content: center;
    background: rgba(0,0,0,.55); color: var(--ink); text-decoration: none;
    font-size: 14px; line-height: 1; opacity: 0; transition: opacity .12s;
  }
  .vid:hover .dl { opacity: 1; }
  .vid .dl:hover { background: var(--accent); color: #14171c; }
  .vid .cap { padding: 6px 10px; font-size: 12px; color: var(--mut); border-top: 1px solid var(--line); }
  /* timeline */
  .timeline {
    position: relative; height: 34px; background: #0e1014; border: 1px solid var(--line);
    border-radius: 5px; margin: 6px 0 4px; cursor: pointer; overflow: hidden;
  }
  .timeline .clip {
    position: absolute; top: 0; bottom: 0; background: rgba(63,185,80,.32);
    border-left: 2px solid var(--clip); border-right: 2px solid var(--clip);
  }
  .timeline .clip .lbl { font-size: 10px; color: var(--clip); padding: 1px 4px; }
  .timeline .pendmark { position: absolute; top: 0; bottom: 0; width: 2px; background: var(--pend); }
  .timeline .playhead { position: absolute; top: 0; bottom: 0; width: 2px; background: var(--play); pointer-events: none; }
  .tlabels { display: flex; justify-content: space-between; color: var(--mut); font-size: 11px; }
  .controls { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; margin: 14px 0; }
  button {
    font: inherit; font-size: 14px; padding: 9px 16px; border-radius: 6px;
    border: 1px solid var(--line); background: var(--panel); color: var(--ink); cursor: pointer;
  }
  button:hover { border-color: var(--mut); }
  button.in { border-color: var(--pend); color: var(--pend); font-weight: 700; }
  button.out { background: var(--clip); color: #0d1117; border-color: var(--clip); font-weight: 700; }
  button.ghost { color: var(--mut); }
  select { font: inherit; padding: 8px; border-radius: 6px; background: var(--panel); color: var(--ink); border: 1px solid var(--line); }
  .cliplist { margin-top: 8px; }
  .cliprow { display: flex; align-items: center; gap: 10px; font-size: 13px; padding: 5px 0; border-bottom: 1px solid var(--line); }
  .cliprow .tag { color: var(--clip); }
  .cliprow button { padding: 3px 9px; font-size: 12px; }
  .hint { color: var(--mut); font-size: 12px; margin-top: 14px; }
  .howto { background: #16202b; border: 1px solid var(--line); border-left: 3px solid var(--play);
           padding: 10px 14px; border-radius: 6px; font-size: 13px; line-height: 1.6; margin: 6px 0 12px; }
  .howto b { color: var(--ink); }
  .howto .ico { color: var(--clip); }
  .banner { background: #3a2a1a; border: 1px solid var(--accent); color: var(--accent); padding: 8px 12px; border-radius: 6px; font-size: 13px; margin-bottom: 14px; }
  kbd { background:#0e1014; border:1px solid var(--line); border-radius:4px; padding:0 5px; font-size:11px; }
</style>
</head>
<body>
<header>
  <h1>EPISODE SPLITTER</h1>
  <span class="stat"><b id="total">0</b> episodes</span>
  <span class="stat">sub-episodes (clips) <b class="c" id="nclips">0</b></span>
  <span class="stat">speed
    <select id="speed">
      <option value="0.25">0.25x</option>
      <option value="0.5">0.5x</option>
      <option value="1" selected>1x</option>
      <option value="1.5">1.5x</option>
      <option value="2">2x</option>
      <option value="4">4x</option>
    </select>
  </span>
</header>
<div class="wrap">
  <div id="list"></div>
  <div id="viewer"><div class="sub">Loading…</div></div>
</div>
<script>
let EPISODES = [], CAMERAS = [], CUR = 0, SMALL = 0, FPS = 30;
let SPEED = parseFloat(localStorage.getItem('splitSpeed') || '1') || 1;
let PENDING = null;          // pending in-point (fraction) or null
let VIDS = [];               // current <video> elements
let PRIMARY = null;          // reference video for the playhead/marking

async function load() {
  const r = await fetch('/api/episodes');
  const d = await r.json();
  EPISODES = d.episodes; CAMERAS = d.cameras; SMALL = d.small_threshold; FPS = d.fps || 30;
  document.getElementById('total').textContent = EPISODES.length;
  const sel = document.getElementById('speed');
  sel.value = String(SPEED);
  sel.onchange = () => { SPEED = parseFloat(sel.value); localStorage.setItem('splitSpeed', sel.value);
                         applySpeed(); };
  renderList(); renderCounts();
  if (EPISODES.length) show(0);
}

function renderCounts() {
  let n = 0; EPISODES.forEach(e => n += (e.clips ? e.clips.length : 0));
  document.getElementById('nclips').textContent = n;
}

function curEp() { return EPISODES[CUR]; }

function renderList() {
  const el = document.getElementById('list');
  el.innerHTML = '';
  EPISODES.forEach((e, i) => {
    const nc = e.clips ? e.clips.length : 0;
    const row = document.createElement('div');
    row.className = 'ep ' + (nc ? 'has' : '') + (i === CUR ? ' active' : '');
    const small = e.frames < SMALL;
    row.innerHTML = `<span class="dot"></span>`
      + `<span class="name">ep ${String(e.index).padStart(3,'0')}</span>`
      + (small ? `<span class="warn">short!</span>` : '')
      + `<span class="sz">${nc ? nc + ' clip' + (nc>1?'s':'') : e.frames + 'f'}</span>`;
    row.onclick = () => show(i);
    el.appendChild(row);
  });
}

// Map between the primary video's real time and the episode fraction [0..1].
// applyWindow stores the scaled real window [_lo,_hi] on each video element.
function frac() {
  const v = PRIMARY;
  if (!v || !(v._hi > v._lo)) return 0;
  return Math.max(0, Math.min(1, (v.currentTime - v._lo) / (v._hi - v._lo)));
}
function seekFrac(f) {
  f = Math.max(0, Math.min(1, f));
  VIDS.forEach(v => { if (v._hi > v._lo) { try { v.currentTime = v._lo + f * (v._hi - v._lo); } catch(e){} } });
}

// 1x = real-time. The video is a time-lapse: it plays its window (_hi-_lo real
// seconds) but the robot actually moved for real_dur seconds. So scale the rate
// by (window / real_dur), then multiply by the user's SPEED. Clamp to what
// browsers allow (~0.0625x..16x).
function rtRate(v) {
  const e = curEp();
  const real = e ? e.real_dur : 0;
  const win = v._hi - v._lo;
  let r = SPEED;
  if (real > 0 && win > 0) r = SPEED * (win / real);
  return Math.max(0.0625, Math.min(16, r));
}
function applySpeed() { VIDS.forEach(v => { try { v.playbackRate = rtRate(v); } catch(e){} }); }

function applyWindow(video, from, to, fspan) {
  const rescale = () => {
    const d = video.duration;
    if (isFinite(d) && d > 0) {
      const s = (fspan > 0) ? (d / fspan) : 1;     // metadata-sec -> real-sec
      video._lo = Math.max(0, from * s);
      video._hi = (to > from) ? Math.min(d, to * s) : d;
    } else { video._lo = 0; video._hi = 0; }
  };
  video._lo = 0; video._hi = 0;
  video.addEventListener('loadedmetadata', () => {
    rescale();
    try { video.playbackRate = rtRate(video); } catch(e){}
    try { video.currentTime = video._lo; } catch(e){}
  });
  // browsers can reset playbackRate when (re)starting playback — re-assert it
  video.addEventListener('play', () => { try { video.playbackRate = rtRate(video); } catch(e){} });
  // loop within the episode window
  video.addEventListener('timeupdate', () => {
    if (video._hi > video._lo && video.currentTime >= video._hi - 0.03) {
      try { video.currentTime = video._lo; video.play().catch(()=>{}); } catch(e){}
    }
  });
}

function fmtTime(sec) {
  if (!isFinite(sec)) sec = 0;
  const m = Math.floor(sec / 60), s = sec - m * 60;
  return `${m}:${s.toFixed(1).padStart(4,'0')}`;
}

function show(i) {
  CUR = i; PENDING = null;
  const e = curEp();
  const v = document.getElementById('viewer');
  const small = e.frames < SMALL;
  let vids = CAMERAS.map(cam => {
    const seg = e.seg[cam] || [0, 0, 0];
    const src = `/video?cam=${encodeURIComponent(cam)}&ep=${e.index}`;
    const fname = `ep${String(e.index).padStart(3,'0')}_${cam}.mp4`;
    return `<div class="vid">
       <video data-from="${seg[0]}" data-to="${seg[1]}" data-fspan="${seg[2]}"
              src="${src}" autoplay muted preload="metadata"></video>
       <a class="dl" href="${src}" download="${fname}" title="Download ${fname}">⬇</a>
       <div class="cap">${cam}</div>
     </div>`;
  }).join('');
  v.innerHTML =
    `<h2>Episode ${String(e.index).padStart(3,'0')}</h2>`
    + `<div class="sub">${CAMERAS.length} camera(s) · ${e.frames} frames · ${fmtTime(e.real_dur)} real-time (1x)`
    + (e.tasks && e.tasks.length ? ` · task: ${e.tasks.join(', ')}` : '') + `</div>`
    + (small ? `<div class="banner">⚠ Very short episode — likely empty/truncated.</div>` : '')
    + `<div class="vids">${vids}</div>`
    + `<div class="timeline" id="tl"></div>`
    + `<div class="tlabels"><span id="tlcur">0:00.0</span><span id="tlend">0:00.0</span></div>`
    + `<div class="howto">`
    + `<b>Splitting this episode into sub-episodes:</b> scrub/play to where a good segment `
    + `<i>starts</i> and press <kbd>I</kbd> (set IN); scrub to where it <i>ends</i> and press <kbd>O</kbd> (set OUT). `
    + `That creates one <span class="ico">▮ clip</span> (a green span on the bar below the playhead). `
    + `Mark as many clips as you like across the episode. `
    + `On export, <b>each clip becomes its own separate episode</b>, and everything <i>outside</i> the clips is dropped — `
    + `that's how one recording is split into several. The bar shows the clips and the blue playhead; click it to seek.`
    + `<br><b style="color:var(--play)">Nothing is modified here</b> — your clips are only saved to <code>episode_clips.json</code>. `
    + `The split dataset is created (in a NEW folder, original untouched) only when you run `
    + `<code>python episode_picker.py --dataset … --export &lt;out&gt;</code>.`
    + `</div>`
    + `<div class="controls">`
    + `<button class="ghost" onclick="step(-1)" title="Step back one frame (hold Shift = 1 second)">◀ frame</button>`
    + `<button class="ghost" onclick="togglePlay()" id="playbtn">⏸ pause</button>`
    + `<button class="ghost" onclick="step(1)" title="Step forward one frame (hold Shift = 1 second)">frame ▶</button>`
    + `<button class="in" onclick="setIn()" title="Mark the START of a clip at the current playhead position">⟦ Set IN (I)</button>`
    + `<button class="out" onclick="setOut()" title="Mark the END at the playhead — creates a clip from IN to here">Set OUT ⟧ (O)</button>`
    + `<button class="ghost" onclick="addWhole()" title="Keep the entire episode as ONE clip (export it unsplit, as a single sub-episode)">＋ Whole episode</button>`
    + `<button class="ghost" onclick="delAtPlayhead()" title="Delete the clip that the playhead is currently inside">✗ Delete clip at playhead (X)</button>`
    + `</div>`
    + `<div class="cliplist" id="cliplist"></div>`
    + `<div class="hint">`
    + `<b>Keys:</b> <kbd>I</kbd> set IN · <kbd>O</kbd> set OUT (creates a clip) · <kbd>X</kbd> delete the clip under the playhead · `
    + `<kbd>Space</kbd> play/pause · <kbd>←</kbd>/<kbd>→</kbd> step one frame (<kbd>Shift</kbd> = 1 second) · `
    + `<kbd>[</kbd> / <kbd>]</kbd> previous / next episode. `
    + `Your clips auto-save to episode_clips.json, so you can stop and resume anytime.</div>`;

  VIDS = Array.from(v.querySelectorAll('video'));
  VIDS.forEach(vid => applyWindow(vid, parseFloat(vid.dataset.from)||0,
                                  parseFloat(vid.dataset.to)||0, parseFloat(vid.dataset.fspan)||0));
  PRIMARY = VIDS[0] || null;
  if (PRIMARY) {
    PRIMARY.addEventListener('timeupdate', updatePlayhead);
    PRIMARY.addEventListener('loadedmetadata', () => { renderTimeline(); updatePlayhead(); });
  }
  document.getElementById('tl').onclick = (ev) => {
    const r = ev.currentTarget.getBoundingClientRect();
    seekFrac((ev.clientX - r.left) / r.width);
  };
  renderTimeline(); renderClipList(); renderList();
  document.querySelector('.ep.active')?.scrollIntoView({block:'nearest'});
  setupCube(e);
}

// ── AprilTag cube overlay ──────────────────────────────────────────────────
// Draws the detected tag centre in each eye of the detection camera plus the
// estimated 3D position, synced to the playhead. No-op if no sidecar exists.
let CUBE = {map:null, video:null, canvas:null, meta:null, frames:0, raf:0};
function stopCube() {
  if (CUBE.raf) { cancelAnimationFrame(CUBE.raf); CUBE.raf = 0; }
  if (CUBE.canvas) CUBE.canvas.remove();
  CUBE = {map:null, video:null, canvas:null, meta:null, frames:0, raf:0};
}
async function setupCube(e) {
  stopCube();
  let d;
  try { d = await (await fetch('/api/cube?ep=' + e.index)).json(); } catch(_) { return; }
  if (!d || !d.camera || !d.dets || !d.dets.length) return;
  const ci = CAMERAS.indexOf(d.camera);
  const vid = (ci >= 0) ? VIDS[ci] : null;
  if (!vid) return;
  const map = new Map();
  d.dets.forEach(r => map.set(r[0], r));
  const cv = document.createElement('canvas');
  cv.className = 'cubeov';
  vid.parentElement.appendChild(cv);
  CUBE = {map, video:vid, canvas:cv, meta:d, frames:e.frames, raf:0};
  loopCube();
}
function loopCube() { drawCube(); CUBE.raf = requestAnimationFrame(loopCube); }
function f3(v) { return (v == null) ? '—' : v.toFixed(3); }
function drawCube() {
  const {video, canvas, meta, map, frames} = CUBE;
  if (!video || !canvas || !meta) return;
  const W = video.clientWidth, H = video.clientHeight;
  if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, W, H);
  if (!(frames > 1)) return;
  const fr = Math.round(frac() * (frames - 1));
  let row = map.get(fr);
  for (let dx = 1; dx <= 3 && !row; dx++) row = map.get(fr - dx) || map.get(fr + dx);
  if (!row) return;
  const sx = W / meta.frame_w, sy = H / meta.frame_h, ew = meta.eye_w;
  ctx.lineWidth = 2; ctx.strokeStyle = '#3fb950'; ctx.font = '12px ui-monospace, monospace';
  // Draw the tag's 4-corner quad. xoff shifts the right eye into the right half.
  const quad = (corners, xoff, label) => {
    if (!corners) return false;
    ctx.beginPath();
    for (let i = 0; i < 4; i++) {
      const x = (corners[2*i] + xoff) * sx, y = corners[2*i+1] * sy;
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    ctx.closePath(); ctx.stroke();
    ctx.fillStyle = '#3fb950';
    ctx.fillText(label, (corners[0] + xoff) * sx + 4, corners[1] * sy - 5);
    return true;
  };
  const cross = (px, py, label) => {
    if (px == null || py == null) return;
    const x = px * sx, y = py * sy;
    ctx.beginPath();
    ctx.moveTo(x-12, y); ctx.lineTo(x+12, y); ctx.moveTo(x, y-12); ctx.lineTo(x, y+12); ctx.stroke();
    ctx.fillStyle = '#3fb950'; ctx.fillText(label, x + 11, y - 11);
  };
  // row = [frame, lcx,lcy, rcx,rcy, sX,sY,sZ, pX,pY,pZ, leftCorners, rightCorners]
  if (!quad(row[11], 0, 'L')) cross(row[1], row[2], 'L');
  if (!quad(row[12], ew, 'R')) cross(row[3] != null ? ew + row[3] : null, row[4], 'R');
  let txt = (row[7] != null)
    ? `cube  stereo XYZ ${f3(row[5])} ${f3(row[6])} ${f3(row[7])} m`
    : 'cube (single eye)';
  if (row[10] != null) txt += `   pnp Z ${f3(row[10])} m`;
  ctx.fillStyle = 'rgba(0,0,0,.65)'; ctx.fillRect(6, 6, Math.min(W-12, 7.2*txt.length + 8), 18);
  ctx.fillStyle = '#9be9a8'; ctx.fillText(txt, 10, 19);
}

// Real-time clock length of the current episode (seconds the robot actually
// moved), used for all on-screen time labels so the clock matches 1x playback.
function epDurReal() { const e = curEp(); return e && e.real_dur ? e.real_dur : 0; }

function renderTimeline() {
  const tl = document.getElementById('tl'); if (!tl) return;
  const e = curEp(); const clips = e.clips || [];
  let html = clips.map((c, idx) =>
    `<div class="clip" style="left:${c[0]*100}%;width:${(c[1]-c[0])*100}%"><span class="lbl">${idx+1}</span></div>`).join('');
  if (PENDING !== null) html += `<div class="pendmark" style="left:${PENDING*100}%"></div>`;
  html += `<div class="playhead" id="ph" style="left:0%"></div>`;
  tl.innerHTML = html;
  const end = epDurReal();
  const te = document.getElementById('tlend'); if (te) te.textContent = fmtTime(end);
}

function updatePlayhead() {
  const ph = document.getElementById('ph'); if (!ph) return;
  const f = frac();
  ph.style.left = (f * 100) + '%';
  const cur = document.getElementById('tlcur');
  if (cur) cur.textContent = fmtTime(f * epDurReal());
}

function renderClipList() {
  const el = document.getElementById('cliplist'); if (!el) return;
  const e = curEp(); const clips = e.clips || []; const N = e.frames;
  if (!clips.length) { el.innerHTML = `<div class="sub">No clips yet. Press <kbd>I</kbd> at a start point, then <kbd>O</kbd> at an end point to create a clip (= one exported sub-episode). This episode won't export anything until it has at least one clip — use <b>＋ Whole episode</b> to keep all of it.</div>`; return; }
  el.innerHTML = clips.map((c, idx) => {
    const f0 = Math.round(c[0]*N), f1 = Math.round(c[1]*N);
    return `<div class="cliprow"><span class="tag">clip ${idx+1}</span>`
      + `<span>frames ${f0}–${f1} (${f1-f0})</span>`
      + `<span class="sub">${fmtTime(c[0]*epDurReal())}–${fmtTime(c[1]*epDurReal())}</span>`
      + `<button onclick="delClip(${idx})">delete</button></div>`;
  }).join('');
}

async function saveClips() {
  const e = curEp();
  await fetch('/api/clips', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ep: e.index, clips: e.clips || []}) });
  renderCounts(); renderList();
}

function setIn()  { PENDING = frac(); renderTimeline(); updatePlayhead(); }
function setOut() {
  if (PENDING === null) { setIn(); return; }   // first press acts as IN
  let a = Math.min(PENDING, frac()), b = Math.max(PENDING, frac());
  PENDING = null;
  if (b - a < 1e-4) { renderTimeline(); return; }
  const e = curEp(); e.clips = (e.clips || []).concat([[+a.toFixed(6), +b.toFixed(6)]]).sort((x,y)=>x[0]-y[0]);
  saveClips(); renderTimeline(); renderClipList();
}
function addWhole() {
  const e = curEp(); e.clips = [[0, 1]]; PENDING = null;
  saveClips(); renderTimeline(); renderClipList();
}
function delClip(idx) {
  const e = curEp(); e.clips.splice(idx, 1);
  saveClips(); renderTimeline(); renderClipList();
}
function delAtPlayhead() {
  const e = curEp(); const f = frac();
  const idx = (e.clips || []).findIndex(c => f >= c[0] && f <= c[1]);
  if (idx >= 0) delClip(idx);
}

function step(d, big) {
  const e = curEp(); if (!e.frames) return;
  const df = (big ? FPS : 1) / e.frames;   // one data frame, or ~1s with Shift
  seekFrac(frac() + d * df);
  setTimeout(updatePlayhead, 30);
}
function togglePlay() {
  const playing = PRIMARY && !PRIMARY.paused;
  VIDS.forEach(v => playing ? v.pause() : v.play().catch(()=>{}));
  const b = document.getElementById('playbtn'); if (b) b.textContent = playing ? '▶ play' : '⏸ pause';
}
function nav(d) { const n = CUR + d; if (n >= 0 && n < EPISODES.length) show(n); }

document.addEventListener('keydown', ev => {
  if (ev.target.tagName === 'SELECT') return;
  const k = ev.key;
  if (k === 'i' || k === 'I') setIn();
  else if (k === 'o' || k === 'O') setOut();
  else if (k === 'x' || k === 'X') delAtPlayhead();
  else if (k === ' ') { ev.preventDefault(); togglePlay(); }
  else if (k === 'ArrowLeft')  { ev.preventDefault(); step(-1, ev.shiftKey); }
  else if (k === 'ArrowRight') { ev.preventDefault(); step(1, ev.shiftKey); }
  else if (k === '[') nav(-1);
  else if (k === ']') nav(1);
});

load();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Split LeRobot v3.0 episodes into sub-episodes (clips).")
    ap.add_argument("--dataset", required=True,
                    help="Path to the v3.0 dataset root (the folder with meta/, data/, videos/).")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--export", metavar="DIR",
                    help="Instead of serving, export marked clips as sub-episodes to a new v3.0 dataset.")
    args = ap.parse_args()

    dataset = Path(args.dataset).expanduser().resolve()
    if not dataset.is_dir():
        raise SystemExit(f"[error] not a directory: {dataset}")

    if args.export:
        clips = load_clips(dataset)
        if not clips:
            raise SystemExit(f"[error] no episode_clips.json in {dataset} — "
                             f"run the splitter and mark some clips first.")
        export_clips(dataset, Path(args.export).expanduser().resolve(), clips)
        return

    Handler = build_app(dataset)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", args.port), Handler) as httpd:
        print(f"\n  Episode splitter running.")
        print(f"  Open in a browser:  http://localhost:{args.port}")
        print(f"  (Ctrl-C to stop. Clips auto-save to episode_clips.json.)\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[stopped] clips saved to", clips_file(dataset))


if __name__ == "__main__":
    main()
