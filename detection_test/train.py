#!/usr/bin/env python3
"""Fine-tune RF-DETR on the "My First Project.coco" cylinder dataset.

Based on Roboflow's notebook:
https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/how-to-finetune-rf-detr-on-detection-dataset.ipynb

The dataset is single-class ("cylinder"); the "object" category in the COCO
export has no annotations and is ignored by the model.

Example:
    python train.py --epochs 30 --batch-size 4 --grad-accum 4
"""
from __future__ import annotations

import argparse

from common import DEFAULT_DATASET_DIR, DEFAULT_OUTPUT_DIR


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset-dir", default=str(DEFAULT_DATASET_DIR),
                   help="Folder containing train/ valid/ test/ COCO splits.")
    p.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR),
                   help="Where checkpoints and logs are written.")
    p.add_argument("--epochs", type=int, default=30)
    # Conservative defaults for an 8 GB GPU (RTX 5060 Ti). Effective batch
    # size = batch-size * grad-accum. Raise batch-size if you have more VRAM.
    p.add_argument("--batch-size", type=int, default=4)
    p.add_argument("--grad-accum", type=int, default=4)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--resolution", type=int, default=560,
                   help="Input resolution; must be a multiple of 56.")
    p.add_argument("--model", choices=["base", "large"], default="base")
    p.add_argument("--early-stopping", action="store_true",
                   help="Stop when validation mAP plateaus.")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # Imported lazily so --help works without the heavy deps installed.
    from rfdetr import RFDETRBase, RFDETRLarge

    Model = RFDETRLarge if args.model == "large" else RFDETRBase
    model = Model()  # loads COCO-pretrained weights to fine-tune from

    print(f"Fine-tuning RF-DETR-{args.model} on {args.dataset_dir}")
    print(f"  epochs={args.epochs}  batch={args.batch_size}  "
          f"grad_accum={args.grad_accum}  lr={args.lr}  res={args.resolution}")

    model.train(
        dataset_dir=args.dataset_dir,
        output_dir=args.output_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        grad_accum_steps=args.grad_accum,
        lr=args.lr,
        resolution=args.resolution,
        early_stopping=args.early_stopping,
    )

    print(f"\nDone. Checkpoints saved in {args.output_dir}/")
    print("Best EMA weights: checkpoint_best_ema.pth")


if __name__ == "__main__":
    main()
