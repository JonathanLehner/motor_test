#!/usr/bin/env python3
"""
Generate and use a ChArUco board for side-by-side stereo camera calibration.

Typical workflow:

  1. Print the matching board:
       .venv/bin/python stereo_charuco_calibrate.py make-board

  2. Capture calibration frames at the same camera mode used for recording:
       .venv/bin/python stereo_charuco_calibrate.py capture --camera 0 --width 2560 --height 720

  3. Calibrate from the saved combined side-by-side frames:
       .venv/bin/python stereo_charuco_calibrate.py calibrate --frames calibration_frames

The output JSON contains top-level fx/fy/cx/cy/baseline values accepted by
apriltag_cube_pose.py --calib, plus full left/right matrices for later use.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

try:
    from PIL import Image
except ImportError:  # pragma: no cover - pillow is present in the repo venv.
    Image = None


DICTS = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_") and isinstance(getattr(cv2.aruco, name), int)
}


@dataclass(frozen=True)
class BoardSpec:
    squares_x: int = 6
    squares_y: int = 8
    square_length_m: float = 0.030
    marker_length_m: float = 0.022
    dictionary: str = "DICT_5X5_100"


def make_board(spec: BoardSpec) -> cv2.aruco.CharucoBoard:
    if spec.dictionary not in DICTS:
        raise SystemExit(f"unknown dictionary {spec.dictionary!r}; try one of: {sorted(DICTS)[:12]} ...")
    dictionary = cv2.aruco.getPredefinedDictionary(DICTS[spec.dictionary])
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y),
        spec.square_length_m,
        spec.marker_length_m,
        dictionary,
    )


def detector_for(board: cv2.aruco.CharucoBoard) -> cv2.aruco.CharucoDetector:
    params = cv2.aruco.CharucoParameters()
    return cv2.aruco.CharucoDetector(board, params)


def detect_charuco(
    detector: cv2.aruco.CharucoDetector,
    image: np.ndarray,
    min_corners: int,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _marker_corners, _marker_ids = detector.detectBoard(gray)
    if corners is None or ids is None or len(ids) < min_corners:
        return None, None
    return corners.astype(np.float32), ids.astype(np.int32)


def split_side_by_side(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    h, w = frame.shape[:2]
    if w % 2:
        frame = frame[:, : w - 1]
        w -= 1
    return frame[:, : w // 2], frame[:, w // 2 :]


def draw_status(
    frame: np.ndarray,
    left: tuple[np.ndarray | None, np.ndarray | None],
    right: tuple[np.ndarray | None, np.ndarray | None],
) -> np.ndarray:
    vis = frame.copy()
    eye_w = vis.shape[1] // 2
    for label, offset, det in (("L", 0, left), ("R", eye_w, right)):
        corners, ids = det
        count = 0 if ids is None else len(ids)
        color = (0, 220, 0) if count else (0, 0, 255)
        cv2.putText(vis, f"{label}: {count} ChArUco corners", (offset + 20, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
        if corners is not None and ids is not None:
            shifted = corners.copy()
            shifted[:, :, 0] += offset
            cv2.aruco.drawDetectedCornersCharuco(vis, shifted, ids, color)
    cv2.line(vis, (eye_w, 0), (eye_w, vis.shape[0]), (255, 255, 255), 1)
    return vis


def command_make_board(args: argparse.Namespace) -> None:
    spec = spec_from_args(args)
    board = make_board(spec)

    dpi = args.dpi
    page_w_mm, page_h_mm = (297.0, 210.0) if args.landscape else (210.0, 297.0)
    board_w_mm = spec.squares_x * spec.square_length_m * 1000.0
    board_h_mm = spec.squares_y * spec.square_length_m * 1000.0
    margin_w_mm = page_w_mm - board_w_mm
    margin_h_mm = page_h_mm - board_h_mm
    if margin_w_mm < 0 or margin_h_mm < 0:
        raise SystemExit(
            f"board is {board_w_mm:.1f}x{board_h_mm:.1f} mm, larger than page "
            f"{page_w_mm:.1f}x{page_h_mm:.1f} mm"
        )

    page_px = (round(page_w_mm / 25.4 * dpi), round(page_h_mm / 25.4 * dpi))
    board_px = (round(board_w_mm / 25.4 * dpi), round(board_h_mm / 25.4 * dpi))
    board_img = board.generateImage(board_px, marginSize=0, borderBits=1)
    page = np.full((page_px[1], page_px[0]), 255, dtype=np.uint8)
    x0 = (page_px[0] - board_px[0]) // 2
    y0 = (page_px[1] - board_px[1]) // 2
    page[y0 : y0 + board_px[1], x0 : x0 + board_px[0]] = board_img

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.suffix.lower() == ".pdf":
        if Image is None:
            png = output.with_suffix(".png")
            cv2.imwrite(str(png), page)
            print(f"[warn] Pillow is not installed; wrote PNG instead: {png}")
        else:
            Image.fromarray(page).save(output, "PDF", resolution=float(dpi))
            print(f"[ok] wrote {output}")
    else:
        png = output.with_suffix(".png")
        cv2.imwrite(str(png), page)
        print(f"[ok] wrote {png}")

    meta = output.with_suffix(".json")
    meta.write_text(json.dumps({"board": asdict(spec), "dpi": dpi, "page_mm": [page_w_mm, page_h_mm]}, indent=2) + "\n")
    print(f"[ok] wrote {meta}")
    print(
        "[print] Use actual size / 100% scaling. "
        f"Board outer size must measure {board_w_mm:.1f} x {board_h_mm:.1f} mm."
    )


def command_capture(args: argparse.Namespace) -> None:
    spec = spec_from_args(args)
    board = make_board(spec)
    detector = detector_for(board)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "board.json").write_text(json.dumps(asdict(spec), indent=2) + "\n")

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {args.camera}")

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[camera] requested {args.width}x{args.height}, got {aw}x{ah}")
    print("[capture] press 's' to save, 'q' or Esc to quit")
    if args.auto:
        print(f"[capture] auto-save enabled, target={args.count} frames, interval={args.interval:.2f}s")

    saved = 0
    last_save = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[warn] failed to read frame")
                continue
            left_img, right_img = split_side_by_side(frame)
            left = detect_charuco(detector, left_img, args.min_corners)
            right = detect_charuco(detector, right_img, args.min_corners)
            good = left[1] is not None and right[1] is not None

            key = -1
            if not args.no_display:
                cv2.imshow("stereo ChArUco capture", draw_status(frame, left, right))
                key = cv2.waitKey(1) & 0xFF

            now = time.time()
            should_save = key == ord("s") or (
                args.auto and good and saved < args.count and now - last_save >= args.interval
            )
            if should_save:
                path = out_dir / f"frame_{saved:03d}.jpg"
                cv2.imwrite(str(path), frame)
                saved += 1
                last_save = now
                print(f"[save] {path} ({saved}/{args.count if args.auto else 'manual'})")

            if key in (ord("q"), 27) or (args.auto and saved >= args.count):
                break
            if args.no_display and not args.auto:
                raise SystemExit("--no-display is only useful with --auto")
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"[done] saved {saved} frame(s) in {out_dir}")


def collect_points(
    frames: list[Path],
    board: cv2.aruco.CharucoBoard,
    min_corners: int,
) -> tuple[
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    tuple[int, int],
]:
    detector = detector_for(board)
    board_points = board.getChessboardCorners().astype(np.float32)
    left_obj, left_img, right_obj, right_img = [], [], [], []
    stereo_obj, stereo_left, stereo_right = [], [], []
    image_size: tuple[int, int] | None = None

    for path in frames:
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            print(f"[skip] cannot read {path}")
            continue
        left_frame, right_frame = split_side_by_side(frame)
        image_size = (left_frame.shape[1], left_frame.shape[0])
        lc, li = detect_charuco(detector, left_frame, min_corners)
        rc, ri = detect_charuco(detector, right_frame, min_corners)
        if lc is not None and li is not None:
            ids = li.reshape(-1)
            left_obj.append(board_points[ids])
            left_img.append(lc.reshape(-1, 2))
        if rc is not None and ri is not None:
            ids = ri.reshape(-1)
            right_obj.append(board_points[ids])
            right_img.append(rc.reshape(-1, 2))
        if lc is None or li is None or rc is None or ri is None:
            print(f"[frame] {path.name}: left={0 if li is None else len(li)} right={0 if ri is None else len(ri)} stereo=0")
            continue

        lmap = {int(i): p for i, p in zip(li.reshape(-1), lc.reshape(-1, 2))}
        rmap = {int(i): p for i, p in zip(ri.reshape(-1), rc.reshape(-1, 2))}
        common = sorted(set(lmap) & set(rmap))
        if len(common) < min_corners:
            print(f"[frame] {path.name}: left={len(li)} right={len(ri)} stereo={len(common)}")
            continue
        stereo_obj.append(board_points[np.array(common, dtype=np.int32)])
        stereo_left.append(np.array([lmap[i] for i in common], dtype=np.float32))
        stereo_right.append(np.array([rmap[i] for i in common], dtype=np.float32))
        print(f"[frame] {path.name}: left={len(li)} right={len(ri)} stereo={len(common)}")

    if image_size is None:
        raise SystemExit("no readable calibration frames found")
    return left_obj, left_img, right_obj, right_img, stereo_obj, stereo_left, stereo_right, image_size


def command_calibrate(args: argparse.Namespace) -> None:
    spec = spec_from_args(args)
    board = make_board(spec)
    frames_dir = Path(args.frames)
    frames = sorted(
        p for ext in ("*.jpg", "*.jpeg", "*.png") for p in frames_dir.glob(ext)
    )
    if not frames:
        raise SystemExit(f"no jpg/png frames found in {frames_dir}")

    points = collect_points(frames, board, args.min_corners)
    left_obj, left_img, right_obj, right_img, stereo_obj, stereo_left, stereo_right, image_size = points
    if len(left_obj) < args.min_frames or len(right_obj) < args.min_frames or len(stereo_obj) < args.min_frames:
        raise SystemExit(
            "not enough usable frames: "
            f"left={len(left_obj)} right={len(right_obj)} stereo={len(stereo_obj)}; "
            f"need at least {args.min_frames}"
        )

    flags = cv2.CALIB_RATIONAL_MODEL if args.rational_model else 0
    left_rms, left_k, left_d, _lrvecs, _ltvecs = cv2.calibrateCamera(
        left_obj, left_img, image_size, None, None, flags=flags
    )
    right_rms, right_k, right_d, _rrvecs, _rtvecs = cv2.calibrateCamera(
        right_obj, right_img, image_size, None, None, flags=flags
    )
    stereo_flags = cv2.CALIB_FIX_INTRINSIC
    stereo_rms, _lk, _ld, _rk, _rd, rot, trans, essential, fundamental = cv2.stereoCalibrate(
        stereo_obj,
        stereo_left,
        stereo_right,
        left_k,
        left_d,
        right_k,
        right_d,
        image_size,
        flags=stereo_flags,
    )

    baseline = float(np.linalg.norm(trans))
    out = {
        "fx": float(left_k[0, 0]),
        "fy": float(left_k[1, 1]),
        "cx": float(left_k[0, 2]),
        "cy": float(left_k[1, 2]),
        "baseline": baseline,
        "image_width": image_size[0],
        "image_height": image_size[1],
        "board": asdict(spec),
        "rms": {
            "left": float(left_rms),
            "right": float(right_rms),
            "stereo": float(stereo_rms),
        },
        "left": {
            "camera_matrix": left_k.tolist(),
            "dist_coeffs": left_d.reshape(-1).tolist(),
        },
        "right": {
            "camera_matrix": right_k.tolist(),
            "dist_coeffs": right_d.reshape(-1).tolist(),
        },
        "stereo": {
            "rotation_left_to_right": rot.tolist(),
            "translation_left_to_right": trans.reshape(-1).tolist(),
            "essential": essential.tolist(),
            "fundamental": fundamental.tolist(),
        },
        "frames_used": {
            "left": len(left_obj),
            "right": len(right_obj),
            "stereo": len(stereo_obj),
        },
    }
    output = Path(args.output)
    output.write_text(json.dumps(out, indent=2) + "\n")
    print(f"[ok] wrote {output}")
    print(
        f"[result] left RMS={left_rms:.4f}, right RMS={right_rms:.4f}, "
        f"stereo RMS={stereo_rms:.4f}, baseline={baseline:.5f} m"
    )
    print(f"[use] .venv/bin/python apriltag_cube_pose.py --dataset DATASET --calib {output}")


def spec_from_args(args: argparse.Namespace) -> BoardSpec:
    return BoardSpec(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length_m=args.square_length_mm / 1000.0,
        marker_length_m=args.marker_length_mm / 1000.0,
        dictionary=args.dictionary,
    )


def add_board_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--squares-x", type=int, default=6, help="ChArUco squares across")
    p.add_argument("--squares-y", type=int, default=8, help="ChArUco squares down")
    p.add_argument("--square-length-mm", type=float, default=30.0, help="chess square size in mm")
    p.add_argument("--marker-length-mm", type=float, default=22.0, help="ArUco marker size in mm")
    p.add_argument("--dictionary", default="DICT_5X5_100", help="OpenCV ArUco dictionary name")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    make = sub.add_parser("make-board", help="write a matching printable ChArUco board")
    add_board_args(make)
    make.add_argument("--output", default="charuco_board_6x8_30mm.pdf")
    make.add_argument("--dpi", type=int, default=300)
    make.add_argument("--landscape", action=argparse.BooleanOptionalAction, default=False)
    make.set_defaults(func=command_make_board)

    cap = sub.add_parser("capture", help="capture combined side-by-side calibration frames")
    add_board_args(cap)
    cap.add_argument("--camera", type=int, default=0)
    cap.add_argument("--width", type=int, default=2560)
    cap.add_argument("--height", type=int, default=720)
    cap.add_argument("--output-dir", default="calibration_frames")
    cap.add_argument("--min-corners", type=int, default=12)
    cap.add_argument("--auto", action="store_true", help="auto-save frames where both eyes detect the board")
    cap.add_argument("--count", type=int, default=35)
    cap.add_argument("--interval", type=float, default=0.75)
    cap.add_argument("--no-display", action="store_true")
    cap.set_defaults(func=command_capture)

    cal = sub.add_parser("calibrate", help="calibrate from saved side-by-side frames")
    add_board_args(cal)
    cal.add_argument("--frames", default="calibration_frames")
    cal.add_argument("--output", default="stereo_calib.json")
    cal.add_argument("--min-corners", type=int, default=12)
    cal.add_argument("--min-frames", type=int, default=12)
    cal.add_argument("--rational-model", action="store_true")
    cal.set_defaults(func=command_calibrate)

    return p


def main() -> None:
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
