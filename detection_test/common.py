"""Shared helpers for the RF-DETR train / test / camera scripts.

The Roboflow COCO export defines two categories:
    id 0 -> "object"   (supercategory placeholder, ZERO annotations -> a dud)
    id 1 -> "cylinder" (the only real, annotated class)

Every script here treats the problem as single-class "cylinder" detection and
ignores the dud class entirely.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

# The Roboflow COCO export lives next to these scripts.
DEFAULT_DATASET_DIR = Path(__file__).resolve().parent / "My First Project.coco"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "output"

# The only class we care about.
TARGET_CLASS_ID = 1
TARGET_CLASS_NAME = "cylinder"

# RF-DETR writes several checkpoints; prefer the EMA weights, which usually
# generalise best, then fall back to the others.
CHECKPOINT_PREFERENCE = (
    "checkpoint_best_ema.pth",
    "checkpoint_best_regular.pth",
    "checkpoint_best_total.pth",
    "checkpoint.pth",
)


def resolve_checkpoint(output_dir: os.PathLike | str = DEFAULT_OUTPUT_DIR) -> Path:
    """Return the best available trained checkpoint inside ``output_dir``."""
    output_dir = Path(output_dir)
    for name in CHECKPOINT_PREFERENCE:
        candidate = output_dir / name
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"No RF-DETR checkpoint found in {output_dir}. Run train.py first "
        f"(looked for: {', '.join(CHECKPOINT_PREFERENCE)})."
    )


def class_name(class_id: int) -> str:
    """Human-readable name for a predicted class id."""
    return TARGET_CLASS_NAME if class_id == TARGET_CLASS_ID else f"class_{class_id}"
