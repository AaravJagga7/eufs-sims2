"""Generate pseudo-depth labels for all training images using Depth Anything V2 Small.

Saves uint16 PNG depth maps at depth/<split>/<stem>.png in each dataset folder.
Format: pixel value = metres * 256  (same as KITTI ground truth, readable by MultiTaskDataset).

Supported datasets: speedbump, rdd, idd, bdd  (kitti already has real depth from stereo)

Run on Colab (needs HuggingFace internet access):
    pip install transformers timm -q
    python scripts/generate_depth_labels.py --root data_processed --datasets speedbump rdd idd bdd

Or for a single dataset / split:
    python scripts/generate_depth_labels.py --root data_processed --datasets speedbump --splits train

The KITTI dataset is intentionally skipped — it already has real LiDAR ground-truth depth.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import torch

MODEL_NAME = "depth-anything/Depth-Anything-V2-Small-hf"
DEPTH_PNG_SCALE = 256.0   # metres * 256 stored as uint16
DEPTH_MAX_M = 80.0
BATCH_SIZE = 8             # images per forward pass; reduce to 4 if OOM on Colab free tier

ALL_DATASETS = ["speedbump", "rdd", "idd", "bdd"]   # kitti excluded (real depth exists)
ALL_SPLITS = ["train", "val", "test"]


def load_model(device):
    """Download and load Depth Anything V2 Small from HuggingFace."""
    try:
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        print(f"Loading {MODEL_NAME} ...")
        processor = AutoImageProcessor.from_pretrained(MODEL_NAME)
        model = AutoModelForDepthEstimation.from_pretrained(MODEL_NAME).to(device)
        model.eval()
        print("Model loaded.\n")
        return processor, model
    except Exception as e:
        print(f"ERROR: Could not load model: {e}")
        print("Make sure you have internet access and: pip install transformers timm")
        sys.exit(1)


def predict_batch(imgs_bgr, processor, model, device):
    """Run depth inference on a list of BGR uint8 images.
    Returns list of float32 depth arrays (H, W) in metres."""
    from PIL import Image

    pil_imgs = [Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)) for img in imgs_bgr]
    inputs = processor(images=pil_imgs, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)
        pred = outputs.predicted_depth   # (B, H', W')  – relative depth

    # Resize each prediction to match the original image size, then scale to metres.
    results = []
    for i, img in enumerate(imgs_bgr):
        h, w = img.shape[:2]
        d = pred[i].cpu().numpy()
        d_resized = cv2.resize(d, (w, h), interpolation=cv2.INTER_LINEAR)
        # Depth Anything V2 Small outputs relative (affine-invariant) depth.
        # Scale so that the 95th-percentile maps to DEPTH_MAX_M.
        p95 = float(np.percentile(d_resized, 95))
        if p95 > 0:
            d_metres = d_resized * (DEPTH_MAX_M / p95)
        else:
            d_metres = d_resized
        d_metres = np.clip(d_metres, 0.0, DEPTH_MAX_M)
        results.append(d_metres.astype(np.float32))
    return results


def process_dataset(root, name, splits, processor, model, device, overwrite=False):
    ds_root = os.path.join(root, name)
    splits_path = os.path.join(ds_root, "splits.json")
    if not os.path.exists(splits_path):
        print(f"  SKIP {name}: no splits.json found at {splits_path}")
        return

    with open(splits_path) as f:
        all_splits = json.load(f)

    total_written = 0
    for split in splits:
        stems = all_splits.get(split, [])
        if not stems:
            print(f"  {name}/{split}: empty, skipping.")
            continue

        depth_dir = os.path.join(ds_root, "depth", split)
        os.makedirs(depth_dir, exist_ok=True)

        # Filter to stems that still need processing
        todo = []
        for stem in stems:
            out_path = os.path.join(depth_dir, stem + ".png")
            if overwrite or not os.path.exists(out_path):
                todo.append(stem)

        if not todo:
            print(f"  {name}/{split}: all {len(stems)} depth maps already exist, skipping.")
            continue

        print(f"  {name}/{split}: generating depth for {len(todo)}/{len(stems)} images ...")
        img_dir = os.path.join(ds_root, "images", split)

        for batch_start in range(0, len(todo), BATCH_SIZE):
            batch_stems = todo[batch_start: batch_start + BATCH_SIZE]
            imgs = []
            valid_stems = []
            for stem in batch_stems:
                img_path = os.path.join(img_dir, stem + ".jpg")
                img = cv2.imread(img_path)
                if img is None:
                    print(f"    WARNING: could not read {img_path}, skipping.")
                    continue
                imgs.append(img)
                valid_stems.append(stem)

            if not imgs:
                continue

            depth_maps = predict_batch(imgs, processor, model, device)

            for stem, d_metres in zip(valid_stems, depth_maps):
                out_path = os.path.join(depth_dir, stem + ".png")
                d_uint16 = (d_metres * DEPTH_PNG_SCALE).clip(0, 65535).astype(np.uint16)
                cv2.imwrite(out_path, d_uint16)

            total_written += len(valid_stems)
            done = min(batch_start + BATCH_SIZE, len(todo))
            print(f"    {done}/{len(todo)} done", end="\r")

        print(f"  {name}/{split}: wrote {len(todo)} depth maps.          ")
    print(f"  {name}: total {total_written} depth maps written.\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="data_processed",
                        help="Path to the data_processed folder")
    parser.add_argument("--datasets", nargs="+", default=ALL_DATASETS,
                        help="Which datasets to process (default: all except kitti)")
    parser.add_argument("--splits", nargs="+", default=ALL_SPLITS,
                        help="Which splits to generate (default: train val test)")
    parser.add_argument("--overwrite", action="store_true",
                        help="Regenerate even if depth PNG already exists")
    parser.add_argument("--device", default=None,
                        help="torch device (default: cuda if available, else cpu)")
    args = parser.parse_args()

    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    if "kitti" in args.datasets:
        print("NOTE: kitti has real LiDAR depth — removing it from the list.")
        args.datasets = [d for d in args.datasets if d != "kitti"]

    processor, model = load_model(device)

    for name in args.datasets:
        print(f"=== {name} ===")
        process_dataset(args.root, name, args.splits, processor, model, device, args.overwrite)

    print("All done. Depth maps saved as uint16 PNG (value = metres × 256).")
    print("MultiTaskDataset will automatically detect and use them (has_depth=True).")


if __name__ == "__main__":
    main()
