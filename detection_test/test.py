#!/usr/bin/env python3
"""Evaluate the trained RF-DETR model on the test split and save annotated images.

Runs the model over every image in the test split, keeps only "cylinder"
detections, prints summary stats, and writes side-by-side annotated images to
an output folder so you can eyeball the results.

Example:
    python test.py --threshold 0.5 --max-images 20
"""
from __future__ import annotations

import argparse
from pathlib import Path

from common import (
    DEFAULT_DATASET_DIR,
    DEFAULT_OUTPUT_DIR,
    TARGET_CLASS_ID,
    TARGET_CLASS_NAME,
    resolve_checkpoint,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset-dir", default=str(DEFAULT_DATASET_DIR))
    p.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR),
                   help="Folder containing the trained checkpoint.")
    p.add_argument("--checkpoint", default=None,
                   help="Explicit checkpoint path (defaults to best in output-dir).")
    p.add_argument("--split", default="test", choices=["test", "valid", "train"])
    p.add_argument("--threshold", type=float, default=0.5)
    p.add_argument("--model", choices=["base", "large"], default="base")
    p.add_argument("--max-images", type=int, default=20,
                   help="How many annotated previews to save (-1 for all).")
    p.add_argument("--vis-dir", default=None,
                   help="Where to save annotated images (default: <output-dir>/predictions).")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    import cv2
    import numpy as np
    import supervision as sv
    from PIL import Image
    from rfdetr import RFDETRBase, RFDETRLarge

    checkpoint = args.checkpoint or str(resolve_checkpoint(args.output_dir))
    print(f"Loading checkpoint: {checkpoint}")

    Model = RFDETRLarge if args.model == "large" else RFDETRBase
    model = Model(pretrain_weights=checkpoint)

    split_dir = Path(args.dataset_dir) / args.split
    images = sorted(
        p for p in split_dir.iterdir()
        if p.suffix.lower() in {".jpg", ".jpeg", ".png"} and not p.name.startswith("_")
    )
    if not images:
        raise SystemExit(f"No images found in {split_dir}")

    vis_dir = Path(args.vis_dir) if args.vis_dir else Path(args.output_dir) / "predictions"
    vis_dir.mkdir(parents=True, exist_ok=True)

    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    total_dets = 0
    images_with_dets = 0
    confidences: list[float] = []
    n_saved = 0
    limit = len(images) if args.max_images < 0 else args.max_images

    for i, path in enumerate(images):
        image = Image.open(path).convert("RGB")
        detections = model.predict(image, threshold=args.threshold)

        # Keep only the cylinder class.
        keep = detections.class_id == TARGET_CLASS_ID
        detections = detections[keep]

        n = len(detections)
        total_dets += n
        images_with_dets += int(n > 0)
        confidences.extend(detections.confidence.tolist())

        if n_saved < limit:
            labels = [f"{TARGET_CLASS_NAME} {c:.2f}" for c in detections.confidence]
            frame = np.array(image)[:, :, ::-1].copy()  # RGB -> BGR for cv2
            frame = box_annotator.annotate(frame, detections)
            frame = label_annotator.annotate(frame, detections, labels)
            cv2.imwrite(str(vis_dir / path.name), frame)
            n_saved += 1

    print("\n=== Evaluation summary ===")
    print(f"split:              {args.split}")
    print(f"images:             {len(images)}")
    print(f"images w/ cylinder: {images_with_dets}")
    print(f"total detections:   {total_dets}")
    if confidences:
        import statistics
        print(f"mean confidence:    {statistics.mean(confidences):.3f}")
        print(f"min/max confidence: {min(confidences):.3f} / {max(confidences):.3f}")
    print(f"annotated previews: {n_saved} saved to {vis_dir}/")


if __name__ == "__main__":
    main()
