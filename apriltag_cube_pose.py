#!/usr/bin/env python3
"""
apriltag_cube_pose.py — Estimate the 3D position of an AprilTag-marked cube from
the external (third-person) stereo camera of a LeRobot v3.0 dataset.

What it does
------------
- Reads a LeRobot v3.0 dataset and, for the chosen camera (default the external
  stereo camera `observation.images.cam_1`), decodes every episode's frames.
- Each frame is a side-by-side stereo image: left eye  = columns [0 : W/2],
                                              right eye = columns [W/2 : W].
- Detects tag36h11 AprilTags (the cube marker) in BOTH eyes.
- Estimates the cube position two independent ways:
    * stereo  : triangulate the tag centre from left/right disparity
                (Z = fx * baseline / disparity), giving X/Y/Z in the LEFT-eye
                camera frame, in metres.
    * pnp      : per-eye solvePnP of the tag's 4 corners -> tag-centre tvec,
                in metres, in each eye's camera frame.
- Writes one row per processed video frame to a parquet file.

AprilTag backend
----------------
The AprilRobotics/apriltag C library (https://github.com/AprilRobotics/apriltag)
is not installed in this environment and does not build cleanly on Python 3.14.
OpenCV's `cv2.aruco` ships the *same* tag36h11 detector, so we use that — the
detections are equivalent. No ffmpeg needed; OpenCV decodes the mp4s directly.

Calibration
-----------
No camera calibration exists for this rig yet, so intrinsics are ESTIMATED from
an assumed per-eye horizontal FOV and the baseline is a guess. All metric output
is therefore approximate. When you have a real calibration, pass --calib
calib.json (see --help) to override the estimate; nothing else changes.

Usage
-----
    python apriltag_cube_pose.py --dataset lerobot_dataset_clean
    python apriltag_cube_pose.py --dataset lerobot_dataset_strawberry \
        --tag-size 0.05 --tag-id 0 --baseline 0.06 --hfov 70
    # later, with real calibration:
    python apriltag_cube_pose.py --dataset DS --calib calib.json
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd


# Feature baked into the dataset: one float32 vector per frame, in the LEFT-eye
# camera frame. Missing components are 0.0; trust the *_visible flags.
CUBE_FEATURE = "observation.cube_position"
CUBE_NAMES = [
    "stereo_x", "stereo_y", "stereo_z",      # triangulated position (m)
    "pnp_x", "pnp_y", "pnp_z",               # left-eye solvePnP position (m)
    "left_cx", "left_cy", "right_cx", "right_cy",   # tag-centre pixels per eye
    "left_visible", "right_visible", "stereo_visible",
]


# --------------------------------------------------------------------------
# Dataset scanning (LeRobot v3.0, metadata-driven) — mirrors episode_picker.py
# --------------------------------------------------------------------------

def load_info(dataset: Path) -> dict:
    info_path = dataset / "meta" / "info.json"
    if not info_path.exists():
        raise SystemExit(
            f"[error] {info_path} not found.\n"
            f"        Point --dataset at the dataset root (folder with meta/, data/, videos/)."
        )
    info = json.loads(info_path.read_text())
    ver = str(info.get("codebase_version", "?"))
    if not ver.startswith("v3"):
        print(f"[warn] codebase_version is {ver!r}, not v3.x — this tool expects v3.0 layout.")
    return info


def read_episodes_meta(dataset: Path) -> pd.DataFrame:
    ep_dir = dataset / "meta" / "episodes"
    if not ep_dir.is_dir():
        raise SystemExit(f"[error] {ep_dir} not found — is this really a v3.0 dataset?")
    files = sorted(ep_dir.rglob("file-*.parquet"))
    if not files:
        raise SystemExit(f"[error] no episode metadata parquet under {ep_dir}")
    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    return df.sort_values("episode_index").reset_index(drop=True)


def scan_camera(dataset: Path, info: dict, camera: str) -> dict:
    """ep_index -> {path, from, to, length} for the chosen camera."""
    feats = info.get("features", {})
    if camera not in feats or feats[camera].get("dtype") != "video":
        cams = [k for k, v in feats.items()
                if isinstance(v, dict) and v.get("dtype") == "video"]
        raise SystemExit(f"[error] camera {camera!r} is not a video feature. "
                         f"Available cameras: {cams}")
    video_tmpl = info["video_path"]
    df = read_episodes_meta(dataset)
    out = {}
    for _, r in df.iterrows():
        ep = int(r["episode_index"])
        ci = int(r[f"videos/{camera}/chunk_index"])
        fi = int(r[f"videos/{camera}/file_index"])
        out[ep] = {
            "path": dataset / video_tmpl.format(video_key=camera, chunk_index=ci, file_index=fi),
            "from": float(r[f"videos/{camera}/from_timestamp"]),
            "to":   float(r[f"videos/{camera}/to_timestamp"]),
            "length": int(r["length"]),
        }
    return out


# --------------------------------------------------------------------------
# Camera model
# --------------------------------------------------------------------------

def estimate_intrinsics(eye_w: int, eye_h: int, hfov_deg: float) -> dict:
    """Pinhole intrinsics for one eye from an assumed horizontal FOV."""
    fx = eye_w / (2.0 * np.tan(np.radians(hfov_deg) / 2.0))
    return {"fx": float(fx), "fy": float(fx), "cx": eye_w / 2.0, "cy": eye_h / 2.0}


def load_calib(path: Path) -> dict:
    """
    Load real calibration, overriding the FOV estimate. Expected JSON keys
    (all optional; missing ones fall back to the estimate / CLI defaults):
        {"fx":.., "fy":.., "cx":.., "cy":.., "baseline":..}
    Both eyes are assumed to share intrinsics (single side-by-side sensor).
    """
    return json.loads(Path(path).read_text())


# --------------------------------------------------------------------------
# AprilTag detection + pose
# --------------------------------------------------------------------------

# Two interchangeable AprilTag backends. Both expose .detect(gray, upscale) ->
# {tag_id: corners(4,2) float32} in ORIGINAL (pre-upscale) pixel coordinates, with
# corners in aruco order (TL, TR, BR, BL) so pose/triangulation/overlay match.

class ArucoBackend:
    """OpenCV cv2.aruco tag36h11 detector (the default; no extra dependency)."""

    name = "aruco"

    def __init__(self, family: str):
        dname = "DICT_" + family.upper().replace("TAG", "APRILTAG_") \
            if not family.upper().startswith("APRILTAG") else "DICT_" + family.upper()
        # normalise common spellings: tag36h11 -> DICT_APRILTAG_36h11
        dname = {"tag36h11": "DICT_APRILTAG_36h11",
                 "tag25h9":  "DICT_APRILTAG_25h9",
                 "tag16h5":  "DICT_APRILTAG_16h5",
                 "tagstandard41h12": "DICT_APRILTAG_36h11"}.get(family.lower(), dname)
        if not hasattr(cv2.aruco, dname):
            raise SystemExit(f"[error] OpenCV has no aruco dictionary {dname!r} for family {family!r}")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dname))
        # Tuned for the small (~17px) tags in this downscaled 320x240 stereo eye:
        # a wider adaptive-threshold window finds more tags, subpix sharpens corners.
        # The big win is upscaling the eye before detection (see .detect).
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 10
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._det = cv2.aruco.ArucoDetector(dictionary, params)

    def detect(self, gray, upscale=1):
        if upscale != 1:
            gray = cv2.resize(gray, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = self._det.detectMarkers(gray)
        out = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                out[int(i)] = (c.reshape(4, 2).astype(np.float32) / upscale)
        return out


class PupilBackend:
    """pupil-apriltags (AprilRobotics C library) detector — often finds tags
    aruco misses on small/blurred markers."""

    name = "pupil"

    def __init__(self, family: str):
        try:
            from pupil_apriltags import Detector
        except ImportError:
            raise SystemExit(
                "[error] backend 'pupil' needs pupil-apriltags: pip install pupil-apriltags")
        self._det = Detector(families=family, nthreads=4, quad_decimate=1.0,
                             refine_edges=True, decode_sharpening=0.25)

    def detect(self, gray, upscale=1):
        if upscale != 1:
            gray = cv2.resize(gray, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
        gray = np.ascontiguousarray(gray)
        out = {}
        for d in self._det.detect(gray):
            # pupil corners wrap counter-clockwise (BL, BR, TR, TL); reversing
            # yields aruco's (TL, TR, BR, BL) order.
            corners = d.corners.astype(np.float32)[::-1].copy() / upscale
            out[int(d.tag_id)] = corners
        return out


def make_detector(family: str, backend: str = "aruco"):
    return {"aruco": ArucoBackend, "pupil": PupilBackend}[backend](family)


def solvepnp_tag(corners, tag_size, K):
    """tvec (x,y,z) of the tag centre in the eye's camera frame, metres. None on failure."""
    h = tag_size / 2.0
    # aruco corner order: TL, TR, BR, BL (image), tag plane at z=0
    obj = np.array([[-h,  h, 0], [h,  h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return tvec.reshape(3)


# --------------------------------------------------------------------------
# Per-episode processing
# --------------------------------------------------------------------------

def process_episode(ep, cam, detector, intr, baseline, tag_size, tag_id, swap_eyes, upscale):
    """Yield a result dict per video frame belonging to this episode."""
    path = cam["path"]
    if not path.exists():
        print(f"[warn] episode {ep}: missing video {path}")
        return
    capture = cv2.VideoCapture(str(path))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    # This video file may hold several episodes; this episode spans [from, to] sec.
    start = max(0, int(round(cam["from"] * fps)))
    end = min(total, int(round(cam["to"] * fps))) if cam["to"] > cam["from"] else total
    if end <= start:
        end = total

    K = np.array([[intr["fx"], 0, intr["cx"]],
                  [0, intr["fy"], intr["cy"]],
                  [0, 0, 1]], dtype=np.float64)

    capture.set(cv2.CAP_PROP_POS_FRAMES, start)
    span = max(1, end - start)
    for vf in range(start, end):
        ok, frame = capture.read()
        if not ok:
            break
        h, w = frame.shape[:2]
        half = w // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        left_g, right_g = gray[:, :half], gray[:, half:]
        if swap_eyes:
            left_g, right_g = right_g, left_g

        ldet = detector.detect(left_g, upscale)
        rdet = detector.detect(right_g, upscale)

        # choose tag: explicit id if given, else a tag seen in both eyes, else any in left
        if tag_id is not None:
            tid = tag_id if (tag_id in ldet or tag_id in rdet) else None
        else:
            both = set(ldet) & set(rdet)
            tid = min(both) if both else (min(ldet) if ldet else (min(rdet) if rdet else None))

        row = {
            "episode_index": ep,
            "video_frame": vf - start,
            "frame_frac": (vf - start) / span,
            "n_tags_left": len(ldet),
            "n_tags_right": len(rdet),
            "tag_id": tid if tid is not None else -1,
        }
        for k in ("left_cx", "left_cy", "right_cx", "right_cy", "disparity",
                  "stereo_X", "stereo_Y", "stereo_Z",
                  "pnp_left_X", "pnp_left_Y", "pnp_left_Z",
                  "pnp_right_X", "pnp_right_Y", "pnp_right_Z"):
            row[k] = np.nan
        # 4 tag corners per eye (flattened x0,y0..x3,y3) for the UI overlay; None if unseen
        row["left_corners"] = None
        row["right_corners"] = None

        if tid is not None:
            if tid in ldet:
                lc = ldet[tid].mean(axis=0)
                row["left_cx"], row["left_cy"] = float(lc[0]), float(lc[1])
                row["left_corners"] = ldet[tid].flatten().tolist()
                t = solvepnp_tag(ldet[tid], tag_size, K)
                if t is not None:
                    row["pnp_left_X"], row["pnp_left_Y"], row["pnp_left_Z"] = map(float, t)
            if tid in rdet:
                rc = rdet[tid].mean(axis=0)
                row["right_cx"], row["right_cy"] = float(rc[0]), float(rc[1])
                row["right_corners"] = rdet[tid].flatten().tolist()
                t = solvepnp_tag(rdet[tid], tag_size, K)
                if t is not None:
                    row["pnp_right_X"], row["pnp_right_Y"], row["pnp_right_Z"] = map(float, t)
            # stereo triangulation (assumes horizontally-aligned rectified eyes)
            if tid in ldet and tid in rdet:
                disp = row["left_cx"] - row["right_cx"]
                row["disparity"] = float(disp)
                if disp > 1e-3:
                    Z = intr["fx"] * baseline / disp
                    row["stereo_Z"] = float(Z)
                    row["stereo_X"] = float((row["left_cx"] - intr["cx"]) * Z / intr["fx"])
                    row["stereo_Y"] = float((row["left_cy"] - intr["cy"]) * Z / intr["fy"])
        yield row
    capture.release()


# --------------------------------------------------------------------------
# Backend comparison: how many tags does each detector find?
# --------------------------------------------------------------------------

def compare_backends(episodes, cams, family, swap_eyes, upscale):
    """Run both backends over the same frames and report detection counts."""
    backends = [make_detector(family, "aruco"), make_detector(family, "pupil")]
    # per backend: total tag detections, eyes (left+right) with >=1 tag, frames seen
    tags = {b.name: 0 for b in backends}
    eyes_hit = {b.name: 0 for b in backends}
    n_frames = 0

    for ep in episodes:
        cam = cams[ep]
        path = cam["path"]
        if not path.exists():
            continue
        capture = cv2.VideoCapture(str(path))
        fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        start = max(0, int(round(cam["from"] * fps)))
        end = min(total, int(round(cam["to"] * fps))) if cam["to"] > cam["from"] else total
        if end <= start:
            end = total
        capture.set(cv2.CAP_PROP_POS_FRAMES, start)
        for _ in range(start, end):
            ok, frame = capture.read()
            if not ok:
                break
            n_frames += 1
            h, w = frame.shape[:2]
            half = w // 2
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            left_g, right_g = gray[:, :half], gray[:, half:]
            if swap_eyes:
                left_g, right_g = right_g, left_g
            for b in backends:
                for eye in (left_g, right_g):
                    det = b.detect(eye, upscale)
                    tags[b.name] += len(det)
                    if det:
                        eyes_hit[b.name] += 1
        capture.release()

    n_eyes = n_frames * 2
    print(f"\n[compare] {n_frames} frames ({n_eyes} eye images) over {len(episodes)} episode(s)")
    print(f"          {'backend':<8} {'tags':>8} {'eyes_with_tag':>14} {'eye_hit_rate':>13}")
    for b in backends:
        rate = (100 * eyes_hit[b.name] / n_eyes) if n_eyes else 0.0
        print(f"          {b.name:<8} {tags[b.name]:>8} {eyes_hit[b.name]:>14} {rate:>12.1f}%")


# --------------------------------------------------------------------------
# Bake the result into the dataset (in place, 1:1 by frame index)
# --------------------------------------------------------------------------

def row_to_vec(r) -> np.ndarray:
    """Map one result row (namedtuple) to the 13-value CUBE_FEATURE vector."""
    vals = [r.stereo_X, r.stereo_Y, r.stereo_Z,
            r.pnp_left_X, r.pnp_left_Y, r.pnp_left_Z,
            r.left_cx, r.left_cy, r.right_cx, r.right_cy]
    vec = np.array([0.0 if (v != v) else v for v in vals], dtype=np.float32)  # NaN -> 0
    return np.concatenate([vec, np.array([
        0.0 if (r.left_cx != r.left_cx) else 1.0,
        0.0 if (r.right_cx != r.right_cx) else 1.0,
        0.0 if (r.stereo_Z != r.stereo_Z) else 1.0,
    ], dtype=np.float32)])


def bake_into_dataset(dataset: Path, df: pd.DataFrame):
    """Add CUBE_FEATURE column to every data parquet and register it in info.json."""
    posmap = {(int(r.episode_index), int(r.video_frame)): row_to_vec(r)
              for r in df.itertuples(index=False)}
    zero = np.zeros(len(CUBE_NAMES), dtype=np.float32)

    data_files = sorted(dataset.rglob("data/**/file-*.parquet"))
    if not data_files:
        raise SystemExit(f"[error] no data parquet under {dataset}/data")
    n_rows = n_filled = 0
    for f in data_files:
        d = pd.read_parquet(f)
        col = [posmap.get((int(e), int(fi)), zero)
               for e, fi in zip(d["episode_index"], d["frame_index"])]
        d[CUBE_FEATURE] = col
        d.to_parquet(f, index=False)
        n_rows += len(d)
        n_filled += sum(1 for v in col if v[12])  # stereo_visible
    print(f"[bake] wrote {CUBE_FEATURE} to {len(data_files)} data file(s): "
          f"{n_rows} rows, {n_filled} with a stereo position")

    info_path = dataset / "meta" / "info.json"
    info = json.loads(info_path.read_text())
    info["features"][CUBE_FEATURE] = {
        "dtype": "float32", "shape": [len(CUBE_NAMES)], "names": CUBE_NAMES,
    }
    info_path.write_text(json.dumps(info, indent=2))
    print(f"[bake] registered feature in {info_path}")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Estimate AprilTag-cube 3D position from the external stereo camera "
                    "of a LeRobot v3.0 dataset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--dataset", type=Path, required=True, help="dataset root (has meta/, videos/)")
    p.add_argument("--camera", default="observation.images.cam_1",
                   help="external stereo camera feature key")
    p.add_argument("--family", default="tag36h11", help="AprilTag family")
    p.add_argument("--backend", default="pupil", choices=["aruco", "pupil"],
                   help="detector backend: 'pupil' (pupil-apriltags C library, default; "
                        "finds ~40%% more tags here) or 'aruco' (OpenCV)")
    p.add_argument("--compare", action="store_true",
                   help="run BOTH backends over the frames and report how many tags "
                        "each detects, then continue the pose pipeline with --backend")
    p.add_argument("--tag-id", type=int, default=None,
                   help="cube's tag id; if unset, auto-pick a tag seen in both eyes")
    p.add_argument("--tag-size", type=float, default=0.05,
                   help="tag black-border edge length in METRES (50mm cube face; "
                        "reduce if the printed tag has a white margin)")
    p.add_argument("--hfov", type=float, default=70.0,
                   help="assumed per-eye horizontal FOV (deg) for intrinsics estimate")
    p.add_argument("--baseline", type=float, default=0.06,
                   help="assumed stereo baseline in METRES (GUESS)")
    p.add_argument("--calib", type=Path, default=None,
                   help="JSON with real {fx,fy,cx,cy,baseline} to override estimates")
    p.add_argument("--upscale", type=float, default=3.0,
                   help="upscale each eye by this factor before detection (cubic); "
                        "the tags are tiny here so 3x ~ +65%% detections, 2x is faster")
    p.add_argument("--swap-eyes", action="store_true",
                   help="treat right half as left eye (if the sensor is mirrored)")
    p.add_argument("--episodes", type=int, nargs="*", default=None,
                   help="only process these episode indices")
    p.add_argument("--output", type=Path, default=None,
                   help="sidecar output parquet (default: <dataset>/apriltag_cube_pose.parquet)")
    p.add_argument("--write-dataset", action="store_true",
                   help=f"also add {CUBE_FEATURE} to the dataset's data parquet IN PLACE "
                        f"(1:1 by frame index) and register it in meta/info.json")
    args = p.parse_args()

    info = load_info(args.dataset)
    cams = scan_camera(args.dataset, info, args.camera)
    feat = info["features"][args.camera]
    H, W = feat["shape"][0], feat["shape"][1]
    eye_w, eye_h = W // 2, H

    intr = estimate_intrinsics(eye_w, eye_h, args.hfov)
    baseline = args.baseline
    src = f"estimated (hfov={args.hfov}°, baseline={baseline} m) — APPROXIMATE"
    if args.calib:
        cal = load_calib(args.calib)
        intr.update({k: float(cal[k]) for k in ("fx", "fy", "cx", "cy") if k in cal})
        if "baseline" in cal:
            baseline = float(cal["baseline"])
        src = f"loaded from {args.calib}"

    detector = make_detector(args.family, args.backend)

    print(f"[info] dataset   : {args.dataset}")
    print(f"[info] camera    : {args.camera}  frame {W}x{H} -> eye {eye_w}x{eye_h}")
    print(f"[info] backend   : {args.backend}")
    print(f"[info] family    : {args.family}  tag_size={args.tag_size} m  tag_id={args.tag_id}")
    print(f"[info] intrinsics: fx={intr['fx']:.1f} fy={intr['fy']:.1f} "
          f"cx={intr['cx']:.1f} cy={intr['cy']:.1f}  baseline={baseline} m")
    print(f"[info] calib src : {src}")

    episodes = sorted(cams) if args.episodes is None else [e for e in args.episodes if e in cams]

    if args.compare:
        compare_backends(episodes, cams, args.family, args.swap_eyes, args.upscale)

    rows = []
    for ep in episodes:
        n0 = len(rows)
        hits = 0
        for row in process_episode(ep, cams[ep], detector, intr, baseline,
                                   args.tag_size, args.tag_id, args.swap_eyes, args.upscale):
            rows.append(row)
            if not np.isnan(row["stereo_Z"]):
                hits += 1
        print(f"[ep {ep:>3}] {len(rows) - n0:>5} frames  {hits:>5} with stereo position")

    if not rows:
        raise SystemExit("[error] no frames processed.")

    df = pd.DataFrame(rows)
    out = args.output or (args.dataset / "apriltag_cube_pose.parquet")
    df.to_parquet(out, index=False)
    # companion metadata so the episode_picker UI can place the eye overlays
    out.with_suffix(".meta.json").write_text(json.dumps({
        "camera": args.camera, "frame_w": W, "frame_h": H, "eye_w": eye_w,
    }, indent=2))

    n_det = int((df["tag_id"] >= 0).sum())
    n_stereo = int(df["stereo_Z"].notna().sum())
    print(f"\n[done] {len(df)} frames -> {out}")
    print(f"       tag detected in {n_det} frames ({100*n_det/len(df):.1f}%), "
          f"stereo position in {n_stereo} ({100*n_stereo/len(df):.1f}%)")
    if n_stereo:
        z = df["stereo_Z"].dropna()
        print(f"       stereo Z range: {z.min():.3f}..{z.max():.3f} m (median {z.median():.3f})")
    print("       NOTE: metric values are approximate until --calib provides real calibration.")

    if args.write_dataset:
        bake_into_dataset(args.dataset, df)


if __name__ == "__main__":
    main()
