#!/usr/bin/env python3
"""
render_sim_video.py — Render the Nova5 IK trajectory as a video and add it to the
LeRobot dataset as a new camera feature `observation.images.sim`.

For every frame it sets the arm to the IK joints from ik_track_cube.py, renders
the MuJoCo scene (with the red target marker), and encodes one mp4 per episode.
It then registers the feature in meta/info.json and adds the per-episode video
columns to meta/episodes, so episode_picker.py shows it automatically to the
right of cam_0 / cam_1 (it displays every video feature, sorted by key).

Run with the IK venv (has mink/mujoco/imageio):
    .venv-ik/bin/python render_sim_video.py --dataset lerobot_dataset_grasp

Requires ik_track_cube.py to have been run first (reads ik_track_cube.parquet).
"""

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
import pandas as pd

SIM_REPO = Path("/Users/jonathanlehner/wundercode/robotics/capulabs/simulabs-simulation")
DEFAULT_SCENE = SIM_REPO / "nova5" / "scene_single.xml"
SIM_KEY = "observation.images.sim"
ARM = "left"
JOINT_COLS = [f"{ARM}_joint{i}" for i in range(1, 7)]
JOINT_NAMES = [f"{ARM}/joint{i}" for i in range(1, 7)]


def make_camera(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    return cam


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    p.add_argument("--width", type=int, default=640)    # match cam_0/cam_1 aspect
    p.add_argument("--height", type=int, default=240)
    p.add_argument("--fps", type=float, default=30.0)
    # Top-down view matching cam_1: arm base at the top reaching down, like the
    # human operator (azimuth 180 puts the base at top; elevation near -70).
    p.add_argument("--distance", type=float, default=1.4)
    p.add_argument("--azimuth", type=float, default=180.0)
    p.add_argument("--elevation", type=float, default=-70.0)
    args = p.parse_args()

    sidecar = args.dataset / "ik_track_cube.parquet"
    if not sidecar.exists():
        raise SystemExit(f"[error] {sidecar} not found. Run ik_track_cube.py first.")
    df = pd.read_parquet(sidecar)
    info = json.loads((args.dataset / "meta" / "info.json").read_text())
    fps = float(info.get("fps", args.fps))
    chunks_size = int(info.get("chunks_size", 1000))
    video_tmpl = info["video_path"]

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[model.joint(jn).id] for jn in JOINT_NAMES]
    mid = model.body(f"{ARM}/target").mocapid[0]
    cam = make_camera([-0.3, 0.0, 0.2], args.distance, args.azimuth, args.elevation)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    episodes = sorted(df.episode_index.unique().astype(int))
    ep_meta = {}   # ep -> (chunk, file, from_ts, to_ts)
    for ep in episodes:
        g = df[df.episode_index == ep].sort_values("frame_index")
        q = g[JOINT_COLS].to_numpy(dtype=float)
        tgt = g[["target_x", "target_y", "target_z"]].to_numpy(dtype=float)
        chunk, fidx = ep // chunks_size, ep % chunks_size
        out = args.dataset / video_tmpl.format(video_key=SIM_KEY, chunk_index=chunk, file_index=fidx)
        out.parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(out, fps=fps, codec="libx264",
                                    pixelformat="yuv420p", macro_block_size=16,
                                    output_params=["-crf", "23"])
        mujoco.mj_resetDataKeyframe(model, data, model.key("neutral_pose").id)
        for i in range(len(q)):
            for k, adr in enumerate(qadr):
                data.qpos[adr] = q[i, k]
            data.mocap_pos[mid] = tgt[i]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam)
            writer.append_data(renderer.render())
        writer.close()
        ep_meta[ep] = (chunk, fidx, 0.0, len(q) / fps)
        print(f"[ep {ep:>3}] {len(q):>5} frames -> {out.relative_to(args.dataset)}")

    # Register the feature in info.json (mirror the cam_1 video block).
    info["features"][SIM_KEY] = {
        "dtype": "video",
        "shape": [args.height, args.width, 3],
        "names": ["height", "width", "channel"],
        "info": {"video.fps": fps, "video.codec": "h264", "video.pix_fmt": "yuv420p",
                 "video.is_depth_image": False, "has_audio": False},
    }
    (args.dataset / "meta" / "info.json").write_text(json.dumps(info, indent=2))

    # Add the per-episode video columns to meta/episodes.
    for f in sorted((args.dataset / "meta" / "episodes").rglob("file-*.parquet")):
        e = pd.read_parquet(f)
        eidx = e["episode_index"].astype(int)
        e[f"videos/{SIM_KEY}/chunk_index"] = eidx.map(lambda x: ep_meta.get(x, (0, 0, 0, 0))[0])
        e[f"videos/{SIM_KEY}/file_index"] = eidx.map(lambda x: ep_meta.get(x, (0, 0, 0, 0))[1])
        e[f"videos/{SIM_KEY}/from_timestamp"] = eidx.map(lambda x: ep_meta.get(x, (0, 0, 0.0, 0.0))[2])
        e[f"videos/{SIM_KEY}/to_timestamp"] = eidx.map(lambda x: ep_meta.get(x, (0, 0, 0.0, 0.0))[3])
        e.to_parquet(f, index=False)

    print(f"\n[done] rendered {len(episodes)} episodes -> {SIM_KEY}; registered feature + episode metadata.")


if __name__ == "__main__":
    main()
