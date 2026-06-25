#!/usr/bin/env python3
"""
ik_replay.py — Replay a generated Nova5 IK trajectory in the MuJoCo viewer.

Loads the per-frame joint angles + EE targets produced by ik_track_cube.py and
plays them back on the Nova5 model: the left arm follows the (workspace-mapped)
cube estimate, and the green mocap target shows where it's tracking.

macOS needs the GLFW main thread, so run with mjpython (not python):
    .venv-ik/bin/mjpython ik_replay.py --dataset lerobot_dataset_grasp --episode 0

Keys: the viewer's own controls (Space pause, etc.). Close the window to quit.
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import pandas as pd

SIM_REPO = Path("/Users/jonathanlehner/wundercode/robotics/capulabs/simulabs-simulation")
DEFAULT_SCENE = SIM_REPO / "nova5" / "scene_single.xml"
ARM = "left"
JOINT_COLS = [f"{ARM}_joint{i}" for i in range(1, 7)]   # sidecar column names
JOINT_NAMES = [f"{ARM}/joint{i}" for i in range(1, 7)]   # model joint names


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    p.add_argument("--fps", type=float, default=30.0, help="playback rate")
    p.add_argument("--loop", action="store_true", help="loop the episode")
    # Initial camera — matches render_sim_video.py's top-down view (orbit with mouse).
    p.add_argument("--distance", type=float, default=1.4)
    p.add_argument("--azimuth", type=float, default=180.0)
    p.add_argument("--elevation", type=float, default=-70.0)
    args = p.parse_args()

    sidecar = args.dataset / "ik_track_cube.parquet"
    if not sidecar.exists():
        raise SystemExit(f"[error] {sidecar} not found. Run ik_track_cube.py first.")
    df = pd.read_parquet(sidecar)
    ep = df[df.episode_index == args.episode].sort_values("frame_index")
    if ep.empty:
        raise SystemExit(f"[error] episode {args.episode} not in {sidecar} "
                         f"(have {sorted(df.episode_index.unique())[:10]}...)")
    q = ep[JOINT_COLS].to_numpy(dtype=float)
    tgt = ep[["target_x", "target_y", "target_z"]].to_numpy(dtype=float)
    print(f"[info] episode {args.episode}: {len(q)} frames @ {args.fps} fps")

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("neutral_pose").id)
    qadr = [model.jnt_qposadr[model.joint(jn).id] for jn in JOINT_NAMES]
    mid = model.body(f"{ARM}/target").mocapid[0]

    dt = 1.0 / args.fps
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [-0.3, 0.0, 0.2]
        viewer.cam.distance = args.distance
        viewer.cam.azimuth = args.azimuth
        viewer.cam.elevation = args.elevation
        while viewer.is_running():
            for i in range(len(q)):
                if not viewer.is_running():
                    break
                t0 = time.time()
                for k, adr in enumerate(qadr):
                    data.qpos[adr] = q[i, k]
                data.mocap_pos[mid] = tgt[i]
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(max(0.0, dt - (time.time() - t0)))
            if not args.loop:
                break


if __name__ == "__main__":
    main()
