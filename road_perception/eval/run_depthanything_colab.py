"""Run Depth Anything V2 Small evaluation on KITTI and append rows to results/metrics.csv.

Designed to run on Google Colab where the HF model hub is accessible.

Usage on Colab:
    # 1. Mount Drive and clone / pull latest main
    # 2. pip install transformers timm
    # 3. python eval/run_depthanything_colab.py --data /path/to/data_processed/kitti

The script will:
- Download depth-anything/Depth-Anything-V2-Small-hf from HuggingFace
- Run evaluation on KITTI test split (200 stereo pairs)
- Append 4 metric rows to results/metrics.csv in the same schema as stereosgbm
"""
import argparse
import csv
import os
import sys
from datetime import date

# Allow running from project root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from eval.eval_depth import evaluate_depth
from eval.depth_wrappers import DepthAnythingV2Wrapper

METRICS_CSV = os.path.join(os.path.dirname(__file__), "..", "results", "metrics.csv")
MODEL_NAME = "depth-anything/Depth-Anything-V2-Small-hf"
RUN_NAME = "depthanythingv2_small_baseline"
MODEL_ID = "depthanythingv2_small"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="data_processed/kitti", help="Path to data_processed/kitti")
    parser.add_argument("--split", default="test")
    parser.add_argument("--batch-size", type=int, default=4)
    args = parser.parse_args()

    print(f"Loading {MODEL_NAME} ...")
    model = DepthAnythingV2Wrapper(model_name=MODEL_NAME)
    if model.model is None:
        print("ERROR: model failed to load. Make sure you have internet access and transformers installed.")
        sys.exit(1)

    print(f"Evaluating on KITTI {args.split} split ...")
    metrics = evaluate_depth(model, data_root=args.data, split=args.split, batch_size=args.batch_size)

    print("\nDepth Anything V2 Small results:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")

    # Append to metrics.csv
    today = date.today().isoformat()
    rows = [
        [today, RUN_NAME, MODEL_ID, "kitti", args.split, k, f"{v:.4f}"]
        for k, v in metrics.items()
    ]

    csv_path = METRICS_CSV
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["date", "run_name", "model", "dataset", "split", "metric_name", "value"])
        writer.writerows(rows)

    print(f"\nAppended {len(rows)} rows to {csv_path}")


if __name__ == "__main__":
    main()
