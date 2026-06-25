#!/usr/bin/env python3
"""
ik_track_cube.py — Generate a Nova5 arm trajectory that tracks the AprilTag cube,
and add it to the LeRobot training data.

Pipeline
--------
1. Read the cube position per frame from the sidecar written by
   apriltag_cube_pose.py (stereo XYZ, in the external camera's left-eye frame).
2. Interpolate over frames with no detection (linear; hold at the ends), so every
   frame has a target.
3. Auto-fit: the cube positions live in an uncalibrated camera frame, so we
   range-map them into the Nova5's reachable workspace box (from simulate.py's
   random_ws). This is NOT physically grounded — it just places a reachable,
   visibly-tracking target until real camera->base extrinsics exist. A sensible
   axis permutation is used so the motion reads naturally (see AXIS_MAP).
4. Solve mink IK (position-only FrameTask + posture/limits, warm-started per
   frame) on the Nova5 MuJoCo model -> 6 joint angles per frame.
5. Write a sidecar parquet, and optionally bake two features into the dataset
   in place (1:1 by frame index):
       observation.arm_qpos_ik    [6]  left/joint1..6 (rad)
       observation.ee_pose_target [3]  the mapped IK target (robot base frame, m)

The cube's existing observation.cube_position[...stereo_visible] flag tells you
which frames had a real detection vs. were interpolated.

Usage
-----
    python ik_track_cube.py --dataset lerobot_dataset_clean
    python ik_track_cube.py --dataset lerobot_dataset_clean --write-dataset
"""

import argparse
import json
from pathlib import Path

import mink
import mujoco
import numpy as np
import pandas as pd

SIM_REPO = Path("/Users/jonathanlehner/wundercode/robotics/capulabs/simulabs-simulation")
DEFAULT_SCENE = SIM_REPO / "nova5" / "scene.xml"

# Nova5 left arm (matches ROBOT_CONFIGS["nova5"] in simulate.py).
ARM = "left"
JOINT_NAMES = [f"{ARM}/joint{i}" for i in range(1, 7)]
EE_SITE = f"{ARM}/gripper"
# Reachable workspace box for the left arm (from simulate.py random_ws["left"]).
WORKSPACE = {"x": (-0.30, 0.00), "y": (-0.30, 0.30), "z": (0.20, 0.40)}
# How camera-frame cube axes map onto robot-base axes (sign, source axis).
# Camera (OpenCV): x=right, y=down, z=forward(depth). Robot: x=reach, y=left/right, z=up.
AXIS_MAP = {"x": ("z", +1.0), "y": ("x", +1.0), "z": ("y", -1.0)}

QPOS_FEATURE = "observation.arm_qpos_ik"
TARGET_FEATURE = "observation.ee_pose_target"


# --------------------------------------------------------------------------
# Cube trajectory: load + interpolate + auto-fit
# --------------------------------------------------------------------------

def load_cube_sidecar(dataset: Path) -> pd.DataFrame:
    path = dataset / "apriltag_cube_pose.parquet"
    if not path.exists():
        raise SystemExit(
            f"[error] {path} not found. Run apriltag_cube_pose.py on this dataset first.")
    return pd.read_parquet(path, columns=["episode_index", "video_frame",
                                          "stereo_X", "stereo_Y", "stereo_Z"])


def episode_cube(df: pd.DataFrame, ep: int) -> np.ndarray:
    """(N,3) cube XYZ for an episode, NaN where stereo not detected, frame-ordered."""
    g = df[df.episode_index == ep].sort_values("video_frame")
    xyz = g[["stereo_X", "stereo_Y", "stereo_Z"]].to_numpy(dtype=float)
    return xyz


def interpolate_nan(xyz: np.ndarray) -> np.ndarray:
    """Linear-interpolate NaN gaps per axis; hold nearest at the ends."""
    out = xyz.copy()
    n = len(out)
    idx = np.arange(n)
    for a in range(3):
        col = out[:, a]
        good = ~np.isnan(col)
        if good.sum() == 0:
            out[:, a] = np.nan          # episode has no detections on this axis
        elif good.sum() == 1:
            out[:, a] = col[good][0]
        else:
            out[:, a] = np.interp(idx, idx[good], col[good])
    return out


def fit_mapping(all_valid: np.ndarray) -> dict:
    """Per robot axis: robust source range -> workspace range (2nd..98th pct)."""
    mp = {}
    for r_axis, (s_name, sign) in AXIS_MAP.items():
        s_idx = {"x": 0, "y": 1, "z": 2}[s_name]
        src = sign * all_valid[:, s_idx]
        lo, hi = np.percentile(src, 2), np.percentile(src, 98)
        if hi - lo < 1e-6:
            hi = lo + 1e-6
        mp[r_axis] = {"s_idx": s_idx, "sign": sign, "lo": lo, "hi": hi}
    return mp


def map_to_workspace(xyz: np.ndarray, mp: dict) -> np.ndarray:
    """Map interpolated camera-frame cube XYZ -> robot-base targets (N,3)."""
    out = np.zeros_like(xyz)
    for j, r_axis in enumerate(("x", "y", "z")):
        m = mp[r_axis]
        ws_lo, ws_hi = WORKSPACE[r_axis]
        src = m["sign"] * xyz[:, m["s_idx"]]
        frac = np.clip((src - m["lo"]) / (m["hi"] - m["lo"]), 0.0, 1.0)
        out[:, j] = ws_lo + frac * (ws_hi - ws_lo)
    return out


# --------------------------------------------------------------------------
# IK
# --------------------------------------------------------------------------

class IKSolver:
    def __init__(self, scene: Path):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("neutral_pose").id)
        self.config = mink.Configuration(self.model)
        self.config.update(self.data.qpos)
        self.neutral_q = self.config.q.copy()

        self.task = mink.FrameTask(frame_name=EE_SITE, frame_type="site",
                                   position_cost=1.0, orientation_cost=0.0, lm_damping=1.0)
        self.posture = mink.PostureTask(self.model, cost=1e-4)
        self.posture.set_target_from_configuration(self.config)
        self.tasks = [self.task, self.posture]

        vlim = {jn: np.pi / 3 for jn in JOINT_NAMES}
        self.limits = [mink.ConfigurationLimit(self.model), mink.VelocityLimit(self.model, vlim)]
        self.qadr = [self.model.jnt_qposadr[self.model.joint(jn).id] for jn in JOINT_NAMES]

    def reset(self):
        self.config.update(self.neutral_q)

    def solve_frame(self, target_pos, dt=1.0 / 30, max_iters=150, pos_thresh=2e-3) -> np.ndarray:
        """Warm-started IK to a 3D position target -> 6 joint angles (rad).
        Early-stops on convergence, so warm-started frames take ~1-2 iters; the
        high cap only matters for the cold reach to the first target."""
        T = mink.SE3.from_rotation_and_translation(mink.SO3.identity(), np.asarray(target_pos))
        self.task.set_target(T)
        for _ in range(max_iters):
            try:
                vel = mink.solve_ik(self.config, self.tasks, dt, "daqp",
                                    limits=self.limits, damping=1e-5)
            except mink.exceptions.NoSolutionFound:
                break
            self.config.integrate_inplace(vel, dt)
            if np.linalg.norm(self.task.compute_error(self.config)[:3]) <= pos_thresh:
                break
        return self.config.q[self.qadr].copy()


# --------------------------------------------------------------------------
# Dataset write (in place, 1:1 by frame index) — mirrors apriltag_cube_pose.py
# --------------------------------------------------------------------------

def bake_into_dataset(dataset: Path, qpos_map: dict, target_map: dict):
    data_files = sorted(dataset.rglob("data/**/file-*.parquet"))
    if not data_files:
        raise SystemExit(f"[error] no data parquet under {dataset}/data")
    zq = np.zeros(6, dtype=np.float32)
    zt = np.zeros(3, dtype=np.float32)
    n_rows = 0
    for f in data_files:
        d = pd.read_parquet(f)
        keys = list(zip(d["episode_index"].astype(int), d["frame_index"].astype(int)))
        d[QPOS_FEATURE] = [qpos_map.get(k, zq) for k in keys]
        d[TARGET_FEATURE] = [target_map.get(k, zt) for k in keys]
        d.to_parquet(f, index=False)
        n_rows += len(d)
    info_path = dataset / "meta" / "info.json"
    info = json.loads(info_path.read_text())
    info["features"][QPOS_FEATURE] = {"dtype": "float32", "shape": [6], "names": JOINT_NAMES}
    info["features"][TARGET_FEATURE] = {"dtype": "float32", "shape": [3], "names": ["x", "y", "z"]}
    info_path.write_text(json.dumps(info, indent=2))
    print(f"[bake] wrote {QPOS_FEATURE} + {TARGET_FEATURE} to {len(data_files)} file(s), "
          f"{n_rows} rows; registered in {info_path}")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--scene", type=Path, default=DEFAULT_SCENE, help="Nova5 MuJoCo scene.xml")
    p.add_argument("--output", type=Path, default=None,
                   help="sidecar parquet (default: <dataset>/ik_track_cube.parquet)")
    p.add_argument("--write-dataset", action="store_true",
                   help=f"also add {QPOS_FEATURE} + {TARGET_FEATURE} to the dataset in place")
    args = p.parse_args()

    df = load_cube_sidecar(args.dataset)
    episodes = sorted(df.episode_index.unique().astype(int))

    # Pass 1: global auto-fit mapping over all real detections.
    valid = df[["stereo_X", "stereo_Y", "stereo_Z"]].to_numpy(dtype=float)
    valid = valid[~np.isnan(valid).any(axis=1)]
    if len(valid) < 2:
        raise SystemExit("[error] not enough cube detections to fit a workspace mapping.")
    mp = fit_mapping(valid)
    print(f"[info] {len(valid)} detected frames; fitting to workspace {WORKSPACE}")

    # Pass 2: per-episode interpolate -> map -> IK (warm-started).
    solver = IKSolver(args.scene)
    qpos_map, target_map, rows = {}, {}, []
    for ep in episodes:
        xyz = episode_cube(df, ep)
        n_det = int((~np.isnan(xyz).any(axis=1)).sum())
        targets = map_to_workspace(interpolate_nan(xyz), mp)
        if np.isnan(targets).any():     # episode had zero detections -> hold workspace centre
            centre = np.array([sum(WORKSPACE[a]) / 2 for a in ("x", "y", "z")])
            targets = np.tile(centre, (len(xyz), 1))
        solver.reset()
        for fr in range(len(targets)):
            q = solver.solve_frame(targets[fr]).astype(np.float32)
            t = targets[fr].astype(np.float32)
            qpos_map[(ep, fr)] = q
            target_map[(ep, fr)] = t
            rows.append({"episode_index": ep, "frame_index": fr,
                         **{JOINT_NAMES[i].replace("/", "_"): float(q[i]) for i in range(6)},
                         "target_x": float(t[0]), "target_y": float(t[1]), "target_z": float(t[2])})
        print(f"[ep {ep:>3}] {len(targets):>5} frames  ({n_det} detected, "
              f"{len(targets) - n_det} interpolated)")

    out = args.output or (args.dataset / "ik_track_cube.parquet")
    pd.DataFrame(rows).to_parquet(out, index=False)
    print(f"\n[done] {len(rows)} frames -> {out}")

    if args.write_dataset:
        bake_into_dataset(args.dataset, qpos_map, target_map)


if __name__ == "__main__":
    main()
