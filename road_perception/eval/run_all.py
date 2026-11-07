"""Unified evaluation runner (owned by Person B).

Runs detection, segmentation, depth, and FPS benchmarking for a given model
and appends all results to results/metrics.csv following Rule 6:
date, run_name, model, dataset, split, metric_name, value

Rule 6 metric names:
- mAP50, mAP50_pothole, mAP50_hump
- mIoU, IoU_road
- depth_rmse_0_10, depth_rmse_10_25, depth_rmse_25_50, depth_absrel
- fps, latency_ms, params_M
"""
import argparse
import csv
from datetime import datetime
import os

from eval.benchmark_fps import benchmark_speed
from eval.eval_depth import evaluate_depth
from eval.eval_det import evaluate_detection
from eval.eval_seg import evaluate_seg

METRICS_CSV = "results/metrics.csv"
CSV_COLUMNS = ["date", "run_name", "model", "dataset", "split", "metric_name", "value"]


def record_metric(date_str, run_name, model_name, dataset, split, metric_name, value):
    """Appends a single metric record to results/metrics.csv."""
    os.makedirs(os.path.dirname(METRICS_CSV), exist_ok=True)
    file_exists = os.path.exists(METRICS_CSV) and os.path.getsize(METRICS_CSV) > 0

    with open(METRICS_CSV, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(CSV_COLUMNS)
        writer.writerow([date_str, run_name, model_name, dataset, split, metric_name, f"{value:.4f}"])


def evaluate_all(model, run_name="baseline", model_name="model",
                 det_dataset="data_processed/speedbump",
                 seg_dataset="data_processed/idd",
                 depth_dataset="data_processed/kitti",
                 split="test",
                 benchmark=True):
    date_str = datetime.now().strftime("%Y-%m-%d")
    results = {}

    # 1. Detection
    if os.path.isdir(os.path.join(det_dataset, "images", split)):
        print(f"Evaluating detection on {det_dataset} ({split})...")
        try:
            det_metrics = evaluate_detection(model, data_root=det_dataset, split=split)
            results.update(det_metrics)
            ds_name = os.path.basename(os.path.normpath(det_dataset))
            for k, v in det_metrics.items():
                record_metric(date_str, run_name, model_name, ds_name, split, k, v)
        except Exception as e:
            print(f"Detection evaluation skipped/failed: {e}")

    # 2. Segmentation
    if os.path.isdir(os.path.join(seg_dataset, "images", split)):
        print(f"Evaluating segmentation on {seg_dataset} ({split})...")
        try:
            seg_metrics = evaluate_seg(model, data_root=seg_dataset, split=split)
            results.update(seg_metrics)
            ds_name = os.path.basename(os.path.normpath(seg_dataset))
            for k, v in seg_metrics.items():
                record_metric(date_str, run_name, model_name, ds_name, split, k, v)
        except Exception as e:
            print(f"Segmentation evaluation skipped/failed: {e}")

    # 3. Depth
    if os.path.isdir(os.path.join(depth_dataset, "images", split)):
        print(f"Evaluating depth on {depth_dataset} ({split})...")
        try:
            depth_metrics = evaluate_depth(model, data_root=depth_dataset, split=split)
            results.update(depth_metrics)
            ds_name = os.path.basename(os.path.normpath(depth_dataset))
            for k, v in depth_metrics.items():
                record_metric(date_str, run_name, model_name, ds_name, split, k, v)
        except Exception as e:
            print(f"Depth evaluation skipped/failed: {e}")

    # 4. FPS Benchmark
    if benchmark:
        print("Benchmarking FPS and latency...")
        try:
            speed_metrics = benchmark_speed(model, n_warmup=10, n_runs=50)
            results.update(speed_metrics)
            for k, v in speed_metrics.items():
                record_metric(date_str, run_name, model_name, "hardware", split, k, v)
        except Exception as e:
            print(f"Speed benchmarking skipped/failed: {e}")

    return results


if __name__ == "__main__":
    from eval.depth_wrappers import get_depth_model

    parser = argparse.ArgumentParser()
    parser.add_argument("--run-name", default="baseline_stereo")
    parser.add_argument("--model", default="stereosgbm")
    parser.add_argument("--split", default="test")
    args = parser.parse_args()

    model = get_depth_model(args.model, dataset_root="data_processed/kitti", split=args.split)
    res = evaluate_all(model, run_name=args.run_name, model_name=args.model, split=args.split)
    print("\nAll evaluation results:")
    for k, v in res.items():
        print(f"  {k}: {v:.4f}")
    print(f"\nSaved metrics to {METRICS_CSV}")
