"""Converts the Roboflow 'Pothole and Speed Breaker' dataset to our shared format.

- The download only has a train folder, so we make our own train/val/test.
- Every photo appears twice (Roboflow made an augmented copy with a different
  'rf.<hash>' in the name). We split by PHOTO so both copies always land in the
  same split, and keep just one copy in val/test.
- Classes: 0 pothole -> object 0, 1 speed breaker -> object 1 (checked against file names).

Run from the repo root:
  python -m datasets.converters.speedbump --src "<path>\\Pothole and Speed Breaker Dataset" --out data_processed/speedbump
"""
import argparse
import os
import random
import re

import cv2
import numpy as np

from datasets.contract import SEED
from datasets.converters.common import (IMG_EXT, clean_boxes, make_dirs, read_yolo,
                                        save_sample, split_groups, write_splits)
from datasets.transforms import letterbox

CLASS_MAP = {0: 0, 1: 1}   # Roboflow class -> our object number
PHOTO_RE = re.compile(r"_(?:jpg|jpeg|png)\.rf\.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", default="data_processed/speedbump")
    ap.add_argument("--limit", type=int, default=None,
                    help="convert at most this many images PER SPLIT (for quick tests)")
    args = ap.parse_args()

    # group the files by photo
    photos = {}
    for folder in ("train", "valid", "val", "test"):
        img_dir = os.path.join(args.src, folder, "images")
        if not os.path.isdir(img_dir):
            continue
        for f in sorted(os.listdir(img_dir)):
            stem, ext = os.path.splitext(f)
            if ext.lower() not in IMG_EXT:
                continue
            photo = PHOTO_RE.split(stem)[0]
            photos.setdefault(photo, []).append(
                (stem, os.path.join(img_dir, f), os.path.join(args.src, folder, "labels", stem + ".txt")))
    print(f"found {sum(len(v) for v in photos.values())} images of {len(photos)} photos")

    rng = random.Random(SEED)
    parts = split_groups(sorted(photos), rng)

    make_dirs(args.out, parts, ("images", "labels_det"))
    splits = {s: [] for s in parts}
    for split, photo_ids in parts.items():
        selected = []
        for pid in photo_ids:
            selected += photos[pid] if split == "train" else photos[pid][:1]
        if args.limit:
            selected = rng.sample(selected, min(args.limit, len(selected)))

        counts = {0: 0, 1: 0}
        for stem, img_path, lbl_path in selected:
            img = cv2.imread(img_path)
            if img is None:
                print("could not read", img_path)
                continue
            classes, boxes = read_yolo(lbl_path)
            keep = np.array([int(c) in CLASS_MAP for c in classes], dtype=bool)
            classes = np.array([CLASS_MAP[int(c)] for c in classes[keep]], dtype=np.int64)
            out = letterbox(img, boxes[keep])
            classes, boxes = clean_boxes(classes, out["boxes"])
            save_sample(args.out, split, stem, out["image"], classes, boxes)
            splits[split].append(stem)
            for c in classes:
                counts[int(c)] += 1
        print(f"{split}: {len(splits[split])} images, {counts[0]} pothole boxes, {counts[1]} speed breaker boxes")
    write_splits(args.out, splits)
    print("done ->", args.out)


if __name__ == "__main__":
    main()
