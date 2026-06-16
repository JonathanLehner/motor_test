# Cylinder detection with RF-DETR

Fine-tune, evaluate, and run live camera inference on the `My First Project.coco`
dataset. The dataset is **single-class**: the only annotated class is `cylinder`
(COCO id 1). The `object` category (id 0) in the export has zero annotations and
is ignored everywhere.

Based on Roboflow's
[how-to-finetune-rf-detr-on-detection-dataset](https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/how-to-finetune-rf-detr-on-detection-dataset.ipynb).

## Files
- `train.py` — fine-tune RF-DETR on the COCO splits.
- `test.py` — evaluate on the test split, save annotated preview images.
- `detect_camera.py` — live detection from a webcam.
- `common.py` — shared paths / class config / checkpoint resolution.

## Setup
These scripts run in the shared `motor_test` pipenv environment (the parent
folder's `Pipfile` / in-project `.venv`), which already has the correct
CUDA 12.8 PyTorch for the RTX 5060 Ti (Blackwell). Only the detection deps are
added on top:

```bash
cd ..                 # motor_test (where the Pipfile lives)
pipenv shell          # or prefix the commands below with: pipenv run

pip install rfdetr supervision opencv-python   # torch/torchvision already present

# sanity check
python -c "import torch; print('cuda', torch.cuda.is_available())"
```

Run the scripts below from inside `pipenv shell` (or via `pipenv run python ...`).

## Train
```bash
python train.py --epochs 30 --batch-size 4 --grad-accum 4
```
Defaults are tuned for an 8 GB GPU (effective batch = `batch-size * grad-accum`).
Checkpoints land in `output/`; the best EMA weights are `output/checkpoint_best_ema.pth`.
If you hit CUDA OOM, drop `--batch-size` to 2 and raise `--grad-accum` to 8,
and/or lower `--resolution` (must be a multiple of 56, e.g. 504 or 448).

## Test
```bash
python test.py --threshold 0.5 --max-images 20
```
Prints detection stats and writes annotated previews to `output/predictions/`.

## Live camera
```bash
python detect_camera.py --camera 0 --threshold 0.5
```
Press `q` to quit.
