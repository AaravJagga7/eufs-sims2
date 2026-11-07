# Road Perception – Person B (Data & Evaluation Lead): Handoff

**Date:** Oct 30, 2026  
**Role:** Person B (Data & Evaluation Lead)  
**Branch:** `b/data-pipeline`  
**Latest Commit:** `74c3653` (`feat(bdd): finalize BDD100K converter for per-image JSON annotations and drivable maps`)  
**Test Status:** 38 / 38 passing (`pytest tests/ -v`)

---

## 1. Executive Summary

**All datasets and Stage 1 + Stage 2 tasks assigned to Person B are now 100% complete, verified, and committed.**

1. **All 5 Datasets Converted & Verified:** Roboflow Speedbump, RDD2022, IDD Lite, KITTI Stereo 2015, and BDD100K are fully converted to the unified format (`data_processed/<dataset>/`).
2. **BDD100K Complete:** Converted 3,430 fully labelled images (with 67,516 object bounding boxes across classes 2–7 and 3,430 road segmentation masks). Split cleanly by video ID.
3. **Open Issues Diagnosed & Resolved:**
   - IDD Lite road mask confirmed correct (~29% road pixels per image).
   - RDD2022 eye check verified.
   - BDD100K detection labels unblocked and verified with 160k+ box pixels on `eye_check_bdd_train.png`.
   - `.gitignore` updated to ignore `venv/` and processed datasets.
4. **Stereo & Monocular Baselines Written:** `StereoSGBMWrapper` and `DepthAnythingV2Wrapper` in `eval/depth_wrappers.py` conforming to Rule 5 (`predict()`).
5. **Complete Evaluation Suite Built:** `eval/eval_det.py`, `eval/eval_seg.py`, `eval/eval_depth.py`, `eval/benchmark_fps.py`, and `eval/run_all.py`.
6. **Baseline Numbers Recorded:** StereoSGBM measurements saved to `results/metrics.csv`.
7. **Training Helpers Implemented:** Multitask Batch Mixer in `datasets/samplers.py` (Day 11), Multitask Augmentations in `datasets/transforms.py` (Day 12), and Experiment Logger in `eval/logger.py` (Day 13).

---

## 2. Dataset Inventory

All converted datasets reside in `data_processed/` (git-ignored per Rule 2):

| Dataset | Tasks / Classes | Status | Sample Count & Numbers |
| :--- | :--- | :--- | :--- |
| **Roboflow Speedbump** | Objects 0 (pothole), 1 (hump) | Converted & Tested | 5,503 images (train: 3,840 / val: 450 / test: 451). Boxes line up cleanly. |
| **RDD2022 (India + Japan)** | Object 0 (pothole) | Converted & Tested | Train: 1,710 / val: 327 / test: 375. Split in 100-frame blocks to prevent data leakage. |
| **IDD Lite** | Road Mask (1: road, 0: non-road, 255: ignore) | Converted & Tested | Split by sequence. Train: ~29% average road coverage. |
| **KITTI Stereo 2015** | Depth (testing only): metric depth $Z = \frac{f \cdot B}{d}$ | Converted & Tested | 200 pairs (`*_10.png`). Calibration parsed per frame. Letterboxed to $640 \times 384$. 16-bit PNG depth maps (`metres * 256`). Right stereo images preserved. |
| **BDD100K** | Objects 2–7 & Road Mask | Converted & Tested | 3,430 images (train: 2,395 / val: 518 / test: 517). 67,516 boxes & 3,430 road masks. Split by video ID. |

---

## 3. Code Inventory (Person B Owned)

### Datasets & Converters
- `datasets/transforms.py`:
  - `letterbox`: Padded resizing to $640 \times 384$ without stretching. Moves boxes, road masks, and metric depth maps in step.
  - `augment_multitask` & `MultitaskAugmentor`: Day 12 augmentations (horizontal flip, $0.8\text{--}1.2\times$ zoom, color jitter, blur) preserving label geometry.
- `datasets/multitask_dataset.py`:
  - `MultiTaskDataset`: PyTorch dataset producing standard dictionary conforming to Rule 4 (`image`, `boxes`, `labels`, `seg_mask`, `depth`, `has_*`, `dataset`, `stem`). Supports optional `transform`.
- `datasets/samplers.py`:
  - `MultitaskBatchSampler`: Day 11 batch mixer ensuring each batch comes from one dataset, interleaving them with pothole/speedbump datasets sampled $\sim 2\times$ as often.
  - `ConcatMultiTaskDataset`: Combines multiple datasets with global index tracking.
- `datasets/converters/bdd.py`: Converts BDD100K objects 2–7 and drivable maps.
- `datasets/converters/kitti.py`: Converts KITTI stereo pairs + calibration into contract format.
- `datasets/converters/idd.py`: Converts IDD Lite sequences to binary road masks.
- `datasets/converters/rdd.py`: RDD2022 pothole extraction with 100-frame block splitting.
- `datasets/converters/speedbump.py`: Roboflow speedbump converter.
- `datasets/converters/common.py`: Shared YOLO reader, bounding box cleaners, and directory creators.
- `datasets/eye_check.py`: Visual verification grid generator.

### Evaluation Suite (`eval/`)
- `eval/depth_wrappers.py`:
  - `StereoSGBMWrapper`: OpenCV StereoSGBM baseline on KITTI left/right image pairs.
  - `DepthAnythingV2Wrapper`: Monocular depth baseline wrapper.
  - Conforms to Rule 5: returns `{"boxes": None, "seg": None, "depth": depth}`.
- `eval/eval_depth.py`: Computes `depth_rmse_0_10`, `depth_rmse_10_25`, `depth_rmse_25_50`, and `depth_absrel`.
- `eval/eval_seg.py`: Computes road `mIoU` and `IoU_road` (ignoring 255).
- `eval/eval_det.py`: Computes `mAP50`, `mAP50_pothole`, and `mAP50_hump`.
- `eval/benchmark_fps.py`: Measures `fps`, `latency_ms`, and `params_M` (20 warmup runs, 200 timed runs).
- `eval/run_all.py`: Single command evaluation writing directly to `results/metrics.csv`.
- `eval/logger.py`: Day 13 WandB and local metric logger.

### Tests
- `tests/test_contract.py`: 27 test cases verifying shape, contract, and leakage across all datasets and splits (all pass).
- `tests/test_transforms.py`: 11 test cases verifying letterbox geometry, padding values, and horizontal flip/zoom augmentations (all pass).

---

## 4. Current Baseline Results in `results/metrics.csv`

The stereo vision baseline (OpenCV StereoSGBM) was evaluated on the 200 KITTI test pairs:

```csv
date,run_name,model,dataset,split,metric_name,value
2026-10-30,stereosgbm_baseline,stereosgbm,kitti,test,depth_rmse_0_10,33.1933
2026-10-30,stereosgbm_baseline,stereosgbm,kitti,test,depth_rmse_10_25,29.8304
2026-10-30,stereosgbm_baseline,stereosgbm,kitti,test,depth_rmse_25_50,31.3233
2026-10-30,stereosgbm_baseline,stereosgbm,kitti,test,depth_absrel,1.8499
2026-10-30,stereosgbm_baseline,stereosgbm,hardware,test,fps,34.8306
2026-10-30,stereosgbm_baseline,stereosgbm,hardware,test,latency_ms,28.7104
2026-10-30,stereosgbm_baseline,stereosgbm,hardware,test,params_M,0.0000
```

---

## 5. Reproduction Commands

To reproduce all checks and conversions in the virtual environment:

```powershell
# 1. Run all unit and contract tests (38 tests)
python -m pytest tests/ -v

# 2. Run StereoSGBM baseline evaluation
python -m eval.run_all --run-name stereosgbm_baseline --model stereosgbm --split test

# 3. Generate eye checks for converted datasets
python -m datasets.eye_check data_processed/bdd --split train --n 50
python -m datasets.eye_check data_processed/speedbump --split train --n 50
python -m datasets.eye_check data_processed/rdd --split train --n 50
python -m datasets.eye_check data_processed/idd --split train --n 50
```

---

## 6. Ready for Integration Day 10 with Person A

- Push `b/data-pipeline` to the remote repository.
- Open a Pull Request from `b/data-pipeline` to `main`.
- Upload `data_processed/` to the shared Google Drive folder so Person A can test the data loader directly.
- Ready to run `run_all.py` on Person A's baseline models (`best.pt` / ONNX) as soon as they hand them over.
