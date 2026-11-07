"""Converts the Kaggle copy of RDD2022 (the RDD_SPLIT folder) to our shared format.

- keeps India and Japan only (Japan is reduced to --japan-max images)
- keeps only potholes (-> object 0); other damage types are dropped
- ignores the folder's own train/val/test split. That split is a random shuffle of
  frames, so neighbouring near-identical frames end up in both train and test.
  Instead we pool everything and re-split in BLOCKS of consecutive image numbers.
  (Assumes image numbers follow capture order. Check this by eye.)
- images without potholes are kept as "nothing here" examples, up to --neg-ratio
  times the number of images that do have potholes.

Run from the repo root:
  python -m datasets.converters.rdd --src "<path>\\RDD_SPLIT" --out data_processed/rdd --pothole-class 3
"""
import argparse
import math
import os
import random
import re

import cv2
import numpy as np

from datasets.contract import SEED
from datasets.converters.common import (IMG_EXT, clean_boxes, make_dirs, read_yolo,
                                        save_sample, split_groups, write_splits)
from datasets.transforms import letterbox

COUNTRIES = ("India", "Japan")


def collect(src):
    """All India/Japan images from the three original folders."""
    items = []
    for folder in ("train", "val", "test"):
        img_dir = os.path.join(src, folder, "images")
        if not os.path.isdir(img_dir):
            continue
        for f in sorted(os.listdir(img_dir)):
            stem, ext = os.path.splitext(f)
            numbers = re.findall(r"\d+", stem)
            country = stem.split("_")[0]
            if country not in COUNTRIES or ext.lower() not in IMG_EXT or not numbers:
                continue
            items.append({"country": country, "num": int(numbers[-1]), "stem": stem,
                          "img": os.path.join(img_dir, f),
                          "lbl": os.path.join(src, folder, "labels", stem + ".txt")})
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="the RDD_SPLIT folder")
    ap.add_argument("--out", default="data_processed/rdd")
    ap.add_argument("--pothole-class", type=int, required=True,
                    help="which class number in the RDD labels is the pothole")
    ap.add_argument("--japan-max", type=int, default=3000)
    ap.add_argument("--block", type=int, default=100, help="consecutive images per block")
    ap.add_argument("--neg-ratio", type=float, default=0.25)
    ap.add_argument("--limit", type=int, default=None,
                    help="convert at most this many images PER SPLIT (for quick tests)")
    args = ap.parse_args()

    rng = random.Random(SEED)
    items = collect(args.src)
    print(f"found {len(items)} India/Japan images")

    # 1. cut into blocks of consecutive images, deal whole blocks into splits
    split_items = {"train": [], "val": [], "test": []}
    for country in COUNTRIES:
        mine = sorted((it for it in items if it["country"] == country), key=lambda it: it["num"])
        blocks = [mine[i:i + args.block] for i in range(0, len(mine), args.block)]
        if country == "Japan":
            rng.shuffle(blocks)
            blocks = blocks[:math.ceil(args.japan_max / args.block)]
        for split, chosen in split_groups(blocks, rng).items():
            split_items[split] += [it for block in chosen for it in block]

    # 2. in each split: keep potholes only, then add some empty images
    for split in list(split_items):
        positives, negatives = [], []
        for it in split_items[split]:
            classes, boxes = read_yolo(it["lbl"])
            it["boxes"] = boxes[classes == args.pothole_class]
            (positives if len(it["boxes"]) else negatives).append(it)
        n_neg = min(len(negatives), int(args.neg_ratio * len(positives)))
        selected = sorted(positives + rng.sample(negatives, n_neg), key=lambda it: it["stem"])
        if args.limit:
            selected = rng.sample(selected, min(args.limit, len(selected)))
        split_items[split] = selected

    # 3. letterbox and write
    make_dirs(args.out, split_items, ("images", "labels_det"))
    splits = {s: [] for s in split_items}
    for split, selected in split_items.items():
        n_boxes = n_empty = 0
        for i, it in enumerate(selected):
            img = cv2.imread(it["img"])
            if img is None:
                print("could not read", it["img"])
                continue
            out = letterbox(img, it["boxes"])
            classes, boxes = clean_boxes(np.zeros(len(out["boxes"]), dtype=np.int64), out["boxes"])
            save_sample(args.out, split, it["stem"], out["image"], classes, boxes)
            splits[split].append(it["stem"])
            n_boxes += len(boxes)
            n_empty += len(boxes) == 0
            if (i + 1) % 1000 == 0:
                print(f"  {split}: {i + 1}/{len(selected)}")
        print(f"{split}: {len(splits[split])} images, {n_boxes} pothole boxes, {n_empty} without potholes")
    write_splits(args.out, splits)
    print("done ->", args.out)


if __name__ == "__main__":
    main()
