"""Converts KITTI Stereo 2015 to our shared format (Rule 4 / testing only).

- 200 pairs from the training set (the _10.png files)
- Left camera image from image_2, right camera from image_3
- Ground truth disparity from disp_occ_0 (divided by 256.0)
- Calibration (focal length fx and camera baseline B) from calib_cam_to_cam
- Depth = fx * B / disparity, capped at DEPTH_MAX_M (80 m)
- Letterboxed to 640x384 without stretching
- Output:
    images/test/<stem>.jpg
    depth/test/<stem>.png (uint16, metres * 256)
    images_right/test/<stem>.jpg (right stereo image for StereoSGBM baseline)
    calib/test/<stem>.json (fx, baseline, fb, scale, pad)
    splits.json

Run from the repo root:
  python -m datasets.converters.kitti --src "<path>\\data_scene_flow" --calib "<path>\\data_scene_flow_calib" --out data_processed/kitti
"""
import argparse
import json
import os

import cv2
import numpy as np

from datasets.contract import DEPTH_MAX_M, DEPTH_PNG_SCALE
from datasets.converters.common import make_dirs, write_splits
from datasets.transforms import letterbox


def read_calib(path):
    """Reads calib_cam_to_cam text file to extract fx and camera baseline B."""
    data = {}
    with open(path) as f:
        for line in f:
            if ":" in line:
                key, vals = line.split(":", 1)
                try:
                    data[key.strip()] = [float(x) for x in vals.strip().split()]
                except ValueError:
                    data[key.strip()] = vals.strip()
    p2 = data["P_rect_02"]
    p3 = data["P_rect_03"]
    fx = float(p2[0])
    fb = float(abs(p2[3] - p3[3]))
    baseline = fb / fx
    return {"fx": fx, "baseline": baseline, "fb": fb}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="the data_scene_flow folder")
    ap.add_argument("--calib", required=True, help="the data_scene_flow_calib folder")
    ap.add_argument("--out", default="data_processed/kitti")
    ap.add_argument("--limit", type=int, default=None,
                    help="convert at most this many images (for quick tests)")
    args = ap.parse_args()

    img2_dir = os.path.join(args.src, "training", "image_2")
    img3_dir = os.path.join(args.src, "training", "image_3")
    disp_dir = os.path.join(args.src, "training", "disp_occ_0")
    calib_dir = os.path.join(args.calib, "training", "calib_cam_to_cam")

    files = sorted([f for f in os.listdir(disp_dir) if f.endswith("_10.png")])
    if args.limit:
        files = files[:args.limit]
    print(f"Found {len(files)} KITTI test pairs (_10 files)")

    split = "test"
    make_dirs(args.out, [split], ("images", "depth"))
    os.makedirs(os.path.join(args.out, "images_right", split), exist_ok=True)
    os.makedirs(os.path.join(args.out, "calib", split), exist_ok=True)

    splits = {"train": [], "val": [], "test": []}

    valid_depth_pixels = 0
    total_pixels = 0

    for i, fname in enumerate(files):
        stem = os.path.splitext(fname)[0]
        idx = stem.split("_")[0]

        img_left_path = os.path.join(img2_dir, fname)
        img_right_path = os.path.join(img3_dir, fname)
        disp_path = os.path.join(disp_dir, fname)
        calib_file = os.path.join(calib_dir, f"{idx}.txt")

        img_left = cv2.imread(img_left_path)
        img_right = cv2.imread(img_right_path)
        disp_raw = cv2.imread(disp_path, cv2.IMREAD_UNCHANGED)

        if img_left is None or disp_raw is None:
            print(f"Error reading {fname}")
            continue

        calib_info = read_calib(calib_file)
        fb = calib_info["fb"]

        # Convert disparity to depth in metres
        # In KITTI, disp_raw is uint16 where disp = disp_raw / 256.0
        disp_px = disp_raw.astype(np.float32) / 256.0
        valid_mask = disp_px > 0
        depth_m = np.zeros_like(disp_px, dtype=np.float32)
        depth_m[valid_mask] = np.clip(fb / disp_px[valid_mask], 0.0, DEPTH_MAX_M)

        # Letterbox left image and depth map
        out_left = letterbox(img_left, depth=depth_m)
        cv2.imwrite(os.path.join(args.out, "images", split, stem + ".jpg"),
                    out_left["image"], [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Save depth as uint16 PNG: metres * DEPTH_PNG_SCALE (0 = no value)
        depth_png = np.clip(np.round(out_left["depth"] * DEPTH_PNG_SCALE), 0, 65535).astype(np.uint16)
        cv2.imwrite(os.path.join(args.out, "depth", split, stem + ".png"), depth_png)

        # Also letterbox right image for stereo baselines
        if img_right is not None:
            out_right = letterbox(img_right)
            cv2.imwrite(os.path.join(args.out, "images_right", split, stem + ".jpg"),
                        out_right["image"], [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Save calib with letterbox scale and pad
        calib_record = dict(calib_info)
        calib_record["scale"] = out_left["scale"]
        calib_record["pad"] = out_left["pad"]
        with open(os.path.join(args.out, "calib", split, stem + ".json"), "w") as f:
            json.dump(calib_record, f, indent=2)

        splits[split].append(stem)
        valid_depth_pixels += int((depth_png > 0).sum())
        total_pixels += depth_png.size

        if (i + 1) % 50 == 0 or (i + 1) == len(files):
            print(f"  {split}: {i + 1}/{len(files)} converted")

    write_splits(args.out, splits)
    pct = (valid_depth_pixels / max(1, total_pixels)) * 100
    print(f"done -> {args.out} ({len(splits[split])} images, {pct:.1f}% valid depth pixels)")


if __name__ == "__main__":
    main()
