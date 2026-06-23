#!/usr/bin/env python3
"""Live cylinder detection from a camera using the trained RF-DETR model.

Opens a camera, runs the trained model on each frame, and draws boxes for
"cylinder" detections. Press 'q' to quit.

Example:
    python detect_camera.py --camera 0 --threshold 0.5
"""
from __future__ import annotations

import argparse
import time

from common import (
    DEFAULT_OUTPUT_DIR,
    NUM_CLASSES,
    TARGET_CLASS_ID,
    TARGET_CLASS_NAME,
    resolve_checkpoint,
    silence_load_warnings,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--camera", type=int, default=0, help="Camera index.")
    p.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR),
                   help="Folder containing the trained checkpoint.")
    p.add_argument("--checkpoint", default=None,
                   help="Explicit checkpoint path (defaults to best in output-dir).")
    p.add_argument("--threshold", type=float, default=0.5)
    p.add_argument("--model", choices=["base", "large"], default="base")
    p.add_argument("--width", type=int, default=1280, help="Requested capture width.")
    p.add_argument("--height", type=int, default=720, help="Requested capture height.")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    import warnings

    silence_load_warnings()

    import cv2
    import supervision as sv
    import torch
    from PIL import Image
    from rfdetr import RFDETRBase, RFDETRLarge

    # optimize_for_inference() traces the model; the resolution/query count are
    # fixed, so the "treated as a constant" TracerWarning is expected and benign.
    warnings.filterwarnings("ignore", category=torch.jit.TracerWarning)
    # RFDETRBase is the correct class for this checkpoint's architecture; the
    # deprecation notice is expected and not actionable here.
    warnings.filterwarnings("ignore", message=".*RFDETRBase.*deprecated.*")

    checkpoint = args.checkpoint or str(resolve_checkpoint(args.output_dir))
    print(f"Loading checkpoint: {checkpoint}")

    Model = RFDETRLarge if args.model == "large" else RFDETRBase
    model = Model(pretrain_weights=checkpoint, num_classes=NUM_CLASSES)

    # Warm up / optionally fuse the model for faster inference if supported.
    if hasattr(model, "optimize_for_inference"):
        try:
            model.optimize_for_inference()
        except Exception as exc:  # pragma: no cover - best effort
            print(f"(optimize_for_inference skipped: {exc})")

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}")

    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    print("Running. Press 'q' in the window to quit.")
    prev = time.time()
    fps = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame; stopping.")
                break

            # cv2 gives BGR; RF-DETR expects RGB.
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detections = model.predict(Image.fromarray(rgb), threshold=args.threshold)

            keep = detections.class_id == TARGET_CLASS_ID
            detections = detections[keep]

            labels = [f"{TARGET_CLASS_NAME} {c:.2f}" for c in detections.confidence]
            frame = box_annotator.annotate(frame, detections)
            frame = label_annotator.annotate(frame, detections, labels)

            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - prev, 1e-6))
            prev = now
            cv2.putText(frame, f"{fps:4.1f} FPS  |  {len(detections)} cylinder(s)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RF-DETR cylinder detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
