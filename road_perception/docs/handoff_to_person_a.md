# Road Perception Project — Handoff from Person B to Person A

**Date:** Oct 30, 2026  
**From:** Person B (Data & Evaluation Lead)  
**To:** Person A (Model Lead)  
**Repo:** [github.com/aryahabbuxo/road_perception](https://github.com/aryahabbuxo/road_perception)  
**Main branch:** up to date, all Person B work merged  

---

## What I've Delivered

Everything from the Person B scope is complete and on `main`. Here's what exists and where.

### 1. Dataset Converters — All 5 Datasets Ready

All converters live in `datasets/converters/`. Every dataset follows the shared contract (`datasets/contract.py`): images at `data_processed/<name>/images/<split>/<stem>.jpg`, detection labels at `labels_det/<split>/<stem>.txt` (YOLO format, normalised cx cy w h), segmentation masks at `masks/<split>/<stem>.png` (uint8, 0=bg 255=ignore 1=road), depth maps at `depth/<split>/<stem>.png` (uint16, metres×256).

| Dataset | Converter | Images | Task | Notes |
|---------|-----------|--------|------|-------|
| **speedbump** | `converters/speedbump.py` | 5,503 | Detection | Classes: 0=pothole, 1=speedbump |
| **rdd** | `converters/rdd.py` | 2,412 | Detection | Class 0=pothole, India+Japan subset |
| **idd** | `converters/idd.py` | ~800 | Segmentation | Road mask only (class 1), sequence-split |
| **kitti** | `converters/kitti.py` | 200 stereo pairs | Depth | Right images + calib JSONs stored |
| **bdd** | `converters/bdd.py` | 3,430 | Detection + Seg | Classes 2=car, 3=two-wheeler, 4=bus/truck, 5=person, 6=traffic light, 7=traffic sign; 3,430 road masks |

**Data on your laptop (Person A):** Everything is in `data_processed/` — it's git-ignored so it won't be in the repo.

**Sizes on disk:**
```
speedbump:  0.31 GB (9,483 files)
rdd:        0.12 GB (4,825 files)
bdd:        0.27 GB (10,291 files)
idd:        0.08 GB (3,215 files)
kitti:      0.03 GB (801 files)
```

---

### 2. Training Helpers

| File | What it does |
|------|-------------|
| `datasets/multitask_dataset.py` | `MultiTaskDataset` — loads any converted dataset, returns contract-compliant dicts |
| `datasets/transforms.py` | `letterbox()`, `augment_multitask()`, `MultitaskAugmentor` — letterbox resize + augmentation for all task heads |
| `datasets/samplers.py` | `MultitaskBatchSampler`, `ConcatMultiTaskDataset` — mix datasets across tasks in one training loop |

**How to load a dataset in your training script:**
```python
from datasets.multitask_dataset import MultiTaskDataset
from datasets.transforms import MultitaskAugmentor
from torch.utils.data import DataLoader
from datasets.contract import collate_fn

ds = MultiTaskDataset("data_processed/speedbump", "train", transform=MultitaskAugmentor())
loader = DataLoader(ds, batch_size=16, shuffle=True, collate_fn=collate_fn)

for batch in loader:
    imgs      = batch["img"]          # (B, 3, 384, 640) float32 [0,1]
    targets   = batch["det_targets"]  # (M, 6) [batch_idx, class, cx, cy, w, h]
    masks  = batch["seg_mask"]   # (B, 384, 640) uint8
    depths = batch["depth"]      # (B, 384, 640) float32 metres
```

---

### 3. Evaluation Suite

All eval code is in `eval/`. Run any evaluator standalone or use `run_all.py` for everything at once.

| File | Runs | Output metrics |
|------|------|---------------|
| `eval/eval_det.py` | mAP50 per class | `det_map50_<class>` |
| `eval/eval_seg.py` | mIoU, IoU_road | `seg_miou`, `seg_iou_road` |
| `eval/eval_depth.py` | RMSE by band + AbsRel | `depth_rmse_0_10`, `_10_25`, `_25_50`, `depth_absrel` |
| `eval/benchmark_fps.py` | FPS, latency, param count | `fps`, `latency_ms`, `params_M` |
| `eval/run_all.py` | All of the above | Writes `results/metrics.csv` |
| `eval/logger.py` | WandB + local CSV fallback | — |

**Run everything:**
```bash
python -m eval.run_all --run-name my_model --model stereosgbm --split test
```

All results append to `results/metrics.csv` in a consistent schema (`date, run_name, model, dataset, split, metric_name, value`).

---

### 4. Stereo Depth Baseline — Fixed & Verified

The StereoSGBM baseline is in `eval/depth_wrappers.py`. It was broken (AbsRel 1.85, RMSE 33 m) due to a letterbox scaling bug — **now fixed**.

**Current verified numbers on KITTI test split (200 pairs):**

| Metric | Value |
|--------|-------|
| RMSE 0–10 m | **2.43 m** |
| RMSE 10–25 m | **5.92 m** |
| RMSE 25–50 m | **8.52 m** |
| AbsRel | **0.076 (7.6%)** |
| FPS | 39.7 |

These are in `results/metrics.csv` and committed.

**What the fix was:** OpenCV SGBM returns `disparity × 16` → divide by 16. The disparity lives in letterboxed pixel space (640px wide), but `fb = focal × baseline` was stored in original pixel space (1242px wide) → multiply `fb` by the `scale` factor stored in each `calib/<split>/<stem>.json`.

---

### 5. Test Suite — 38/38 Passing

```bash
pytest tests/ -v
# 38 passed (+ 2 skipped if KITTI/speedbump data absent, + 1 from Person A integration test that needs network)
```

- `tests/test_contract.py` — validates every dataset split against the data contract
- `tests/test_transforms.py` — letterbox geometry, augmentation correctness (skips gracefully without data)

---

### 6. Open PR (Needs Your Review)

**`b/fixes` → `main`:** [github.com/aryahabbuxo/road_perception/pull/new/b/fixes](https://github.com/aryahabbuxo/road_perception/pull/new/b/fixes)

Contains:
1. `test_augment_with_dataset` now skips cleanly on machines without data (was crashing with `FileNotFoundError`)
2. `eval/run_depthanything_colab.py` — Colab runner for Depth Anything V2 Small (see Task A below)

Please review and merge this PR.

---

## What You (Person A) Need to Do Next

### Immediate — Before Training

#### Step 1: Upload the zips to Google Drive
Two flat zips are ready at the project root:
```
speedbump.zip  →  313 MB
rdd.zip        →  117 MB
```
Upload both to **Google Drive → `RoadPerception/data_processed/`**.  
Structure inside each zip: `images/`, `labels_det/`, `splits.json` at the top level — no extra wrapper folder — so Colab can do:
```python
import zipfile
with zipfile.ZipFile("/content/drive/MyDrive/RoadPerception/data_processed/speedbump.zip") as zf:
    zf.extractall("/content/data_processed/speedbump")
```

#### Step 2: On Colab, load the data
```python
from datasets.multitask_dataset import MultiTaskDataset
from datasets.transforms import MultitaskAugmentor
from torch.utils.data import DataLoader
from datasets.contract import collate_fn

ds = MultiTaskDataset("/content/data_processed/speedbump", "train", transform=MultitaskAugmentor())
loader = DataLoader(ds, batch_size=16, shuffle=True, collate_fn=collate_fn)
```

#### Step 3: Merge the `b/fixes` PR
Go to [github.com/aryahabbuxo/road_perception](https://github.com/aryahabbuxo/road_perception) → Pull requests → `b/fixes` → review and merge. This fixes the test crash on your machine.

---

### Soon — Depth Anything V2 Baseline

Run **`eval/run_depthanything_colab.py`** on Colab (it needs network to download weights from HuggingFace, which is why it can't run locally here):

```python
# On Colab:
!pip install transformers timm -q
!python eval/run_depthanything_colab.py --data /content/data_processed/kitti
```

This will:
- Download `depth-anything/Depth-Anything-V2-Small-hf` (~100 MB)
- Evaluate on KITTI test split
- Append 4 rows to `results/metrics.csv`
- Give you the stereo vs. monocular comparison side by side

> [!NOTE]
> KITTI data (35 MB) will need uploading to Drive too before this step. Zip with:
> ```powershell
> Compress-Archive -Path data_processed\kitti\* -DestinationPath kitti.zip -Force
> ```

---

### Later — BDD100K on Kaggle

BDD100K (`data_processed/bdd/`, 277 MB) is too big for free Drive. Recommended path:
1. Create a **private Kaggle Dataset** called `road-perception-bdd`
2. Upload `data_processed/bdd/` there
3. On Colab: Mount Kaggle API key → `kaggle datasets download`

The converter is done; the data is ready locally. Just needs a sharing mechanism.

---

### Class-to-Dataset Mapping (for your multi-task head)

| Class ID | Name | Dataset(s) |
|----------|------|-----------|
| 0 | pothole | speedbump, rdd |
| 1 | speedbump | speedbump |
| 2 | car | bdd |
| 3 | two-wheeler (bicycle, bike, motor) | bdd |
| 4 | bus / truck / train | bdd |
| 5 | person / pedestrian / rider | bdd |
| 6 | traffic light | bdd |
| 7 | traffic sign | bdd |
| road (seg) | — | idd, bdd |
| depth | — | kitti |

---

## File Tree of Everything Person B Delivered

```
datasets/
  contract.py              ← shared data contract (don't modify)
  converters/
    common.py              ← shared letterbox + label writer helpers
    speedbump.py           ← Roboflow converter
    rdd.py                 ← RDD2022 converter
    idd.py                 ← IDD Lite converter
    kitti.py               ← KITTI Stereo 2015 converter
    bdd.py                 ← BDD100K 10k converter
  multitask_dataset.py     ← MultiTaskDataset (use this in training)
  transforms.py            ← letterbox + augmentation
  samplers.py              ← multi-dataset batch sampler

eval/
  depth_wrappers.py        ← StereoSGBMWrapper, DepthAnythingV2Wrapper
  eval_depth.py            ← depth evaluation (RMSE + AbsRel)
  eval_det.py              ← detection evaluation (mAP50)
  eval_seg.py              ← segmentation evaluation (mIoU)
  benchmark_fps.py         ← FPS + latency benchmark
  run_all.py               ← runs all evaluators, writes metrics.csv
  run_depthanything_colab.py ← Depth Anything V2 runner (needs Colab)
  logger.py                ← WandB + CSV logger

tests/
  test_contract.py         ← 28 contract tests
  test_transforms.py       ← 11 transform tests (skip-safe without data)

results/
  metrics.csv              ← stereosgbm baseline numbers (verified)

docs/
  handoff.md               ← in-repo handoff (for future reference)
```

---

## Quick Reference — Key Numbers

| Item | Value |
|------|-------|
| Total images across all datasets | **11,545** |
| Detection boxes (BDD100K) | 67,516 |
| Road mask images (IDD + BDD) | 4,230 |
| KITTI stereo pairs | 200 |
| Test suite | **38 passed** |
| Stereo AbsRel (SGBM, KITTI) | **0.076** |
| Stereo RMSE 0–10 m | **2.43 m** |

---

> [!IMPORTANT]
> The two things blocking your training start are:
> 1. **Upload `speedbump.zip` and `rdd.zip` to Drive** (files are ready at the project root)
> 2. **Merge the `b/fixes` PR** on GitHub

Everything else can happen in parallel with your model work. Good luck! 🚀
