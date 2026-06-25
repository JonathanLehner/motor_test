#!/usr/bin/env python3
"""
camera_test.py — Open the camera(s) and show a live preview.

Useful for checking which camera index is which, and verifying the resolution
the camera ACTUALLY delivers (these are side-by-side stereo cameras, so the
frame is both eyes; per-eye width is half). Mirrors the recorder's capture
setup: MJPG + requested resolution, then reports what came back.

Usage
-----
    python camera_test.py                       # default cam-ids + 2560x720
    python camera_test.py --cam-ids 0           # single camera
    python camera_test.py --cam-ids 0 2 --width 1280 --height 480
Press 'q' (or Esc) in a window to quit.
"""

import argparse

import cv2


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--cam-ids", type=int, nargs="+", default=[0, 2],
                   help="camera indices to open")
    p.add_argument("--width", type=int, default=2560,
                   help="requested combined frame width (side-by-side: both eyes)")
    p.add_argument("--height", type=int, default=720, help="requested frame height")
    p.add_argument("--no-display", action="store_true",
                   help="just print the resolution and exit (no preview window)")
    args = p.parse_args()

    caps = []
    for cid in args.cam_ids:
        cap = cv2.VideoCapture(cid)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not cap.isOpened():
            print(f"[error] cannot open camera {cid}")
            continue
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        flag = "" if (aw, ah) == (args.width, args.height) else "  <-- fell back!"
        print(f"camera {cid}: {aw}x{ah} (side-by-side: {aw // 2}x{ah} per eye) "
              f"@ {fps:.0f} fps{flag}")
        caps.append((cid, cap))

    if not caps:
        raise SystemExit("[error] no cameras opened.")
    if args.no_display:
        for _, cap in caps:
            cap.release()
        return

    print("Showing preview — press 'q' or Esc to quit.")
    try:
        while True:
            for cid, cap in caps:
                ok, frame = cap.read()
                if ok:
                    cv2.imshow(f"camera {cid}", frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
    finally:
        for _, cap in caps:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
