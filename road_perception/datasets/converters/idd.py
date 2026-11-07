"""Converts IDD Lite (the idd20k_lite folder) to our shared format. Road masks only.

- IDD Lite label values: 0 drivable, 1 non-drivable, 2 living things, 3 vehicles,
  4 roadside objects, 5 far objects, 6 sky, 255 unlabelled.
  We write: drivable (0) -> 1 road, everything else (1-6) -> 0 not road, 255 -> 255 ignore.
- Only the official train and val folders have labels. We split by SEQUENCE (route):
    our test = the official val sequences
    our val  = about 20% of the official train sequences
    our train = the rest of the official train sequences

Run from the repo root:
  python -m datasets.converters.idd --src "<path>\\idd-lite\\idd20k_lite" --out data_processed/idd
"""
import argparse
import os
import random

import cv2
import numpy as np

from datasets.contract import SEED
from datasets.converters.common import make_dirs, save_sample, split_groups, write_splits
from datasets.transforms import letterbox

DRIVABLE_ID = 0
IGNORE_ID = 255


def collect(src, folder):
    """Returns {sequence: [(stem, image path, label path), ...]} for one official folder."""
    seqs = {}
    img_root = os.path.join(src, "leftImg8bit", folder)
    for seq in sorted(os.listdir(img_root)):
        for f in sorted(os.listdir(os.path.join(img_root, seq))):
            if not f.endswith("_image.jpg"):
                continue
            idx = f[:-len("_image.jpg")]
            lbl = os.path.join(src, "gtFine", folder, seq, idx + "_label.png")
            if os.path.exists(lbl):
                seqs.setdefault(seq, []).append((f"{seq}_{idx}", os.path.join(img_root, seq, f), lbl))
    return seqs


def to_road_mask(label):
    """IDD label image -> 0 / 1 / 255 road mask (uint8)."""
    if label.ndim == 3:
        label = label[..., 0]
    road = np.zeros(label.shape, np.uint8)
    road[label == DRIVABLE_ID] = 1
    road[label == IGNORE_ID] = 255
    return road


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="the idd20k_lite folder")
    ap.add_argument("--out", default="data_processed/idd")
    ap.add_argument("--limit", type=int, default=None,
                    help="convert at most this many images PER SPLIT (for quick tests)")
    args = ap.parse_args()

    rng = random.Random(SEED)
    train_seqs, val_seqs = collect(args.src, "train"), collect(args.src, "val")
    parts = split_groups(sorted(train_seqs), rng, fracs=(0.8, 0.2, 0.0))
    chosen = {"train": [i for s in parts["train"] for i in train_seqs[s]],
              "val": [i for s in parts["val"] + parts["test"] for i in train_seqs[s]],
              "test": [i for s in sorted(val_seqs) for i in val_seqs[s]]}

    make_dirs(args.out, chosen, ("images", "masks"))
    splits = {s: [] for s in chosen}
    for split, items in chosen.items():
        if args.limit:
            items = rng.sample(items, min(args.limit, len(items)))
        road_fracs = []
        for n, (stem, img_path, lbl_path) in enumerate(items):
            img = cv2.imread(img_path)
            label = cv2.imread(lbl_path, cv2.IMREAD_UNCHANGED)
            if img is None or label is None:
                print("could not read", img_path, lbl_path)
                continue
            if n == 0 and split == "train":
                print(f"first image is {img.shape[1]}x{img.shape[0]} (width x height)")
            road = to_road_mask(label)
            if road.shape != img.shape[:2]:
                road = cv2.resize(road, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            out = letterbox(img, seg_mask=road)
            save_sample(args.out, split, stem, out["image"])
            cv2.imwrite(os.path.join(args.out, "masks", split, stem + ".png"), out["seg_mask"])
            splits[split].append(stem)
            valid = out["seg_mask"] != 255
            road_fracs.append((out["seg_mask"] == 1).sum() / max(1, valid.sum()))
        print(f"{split}: {len(splits[split])} images, road covers on average "
              f"{100 * np.mean(road_fracs):.0f}% of labelled pixels")
    write_splits(args.out, splits)
    print("done ->", args.out)


if __name__ == "__main__":
    main()
