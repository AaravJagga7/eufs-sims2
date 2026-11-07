"""Draws boxes (and road masks, if the dataset has them) on random converted
images and saves them as one picture, so you can check by eye that everything lines up.

Run from the repo root:
  python -m datasets.eye_check data_processed/rdd --split train --n 50
Writes eye_check_<dataset>_<split>.png in the current folder.
"""
import argparse
import os
import random

import cv2
import numpy as np
import yaml

from datasets.contract import SEED
from datasets.converters.common import read_yolo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="e.g. data_processed/rdd")
    ap.add_argument("--split", default="train")
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--cols", type=int, default=5)
    args = ap.parse_args()

    with open("configs/classes.yaml") as f:
        names = yaml.safe_load(f)["detection"]

    img_dir = os.path.join(args.root, "images", args.split)
    stems = sorted(os.path.splitext(f)[0] for f in os.listdir(img_dir))
    random.Random(SEED).shuffle(stems)

    tiles = []
    for stem in stems[:args.n]:
        img = cv2.imread(os.path.join(img_dir, stem + ".jpg"))
        H, W = img.shape[:2]

        mask_path = os.path.join(args.root, "masks", args.split, stem + ".png")
        if os.path.exists(mask_path):
            m = cv2.imread(mask_path, cv2.IMREAD_UNCHANGED)
            overlay = img.copy()
            overlay[m == 1] = (0, 255, 0)      # road = green
            overlay[m == 255] = (255, 0, 0)    # ignored = blue
            img = cv2.addWeighted(img, 0.6, overlay, 0.4, 0)

        lbl_path = os.path.join(args.root, "labels_det", args.split, stem + ".txt")
        classes, boxes = read_yolo(lbl_path)
        for c, (cx, cy, w, h) in zip(classes, boxes):
            p1 = (int((cx - w / 2) * W), int((cy - h / 2) * H))
            p2 = (int((cx + w / 2) * W), int((cy + h / 2) * H))
            cv2.rectangle(img, p1, p2, (0, 0, 255), 2)
            cv2.putText(img, names.get(int(c), str(c)), (p1[0], max(12, p1[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        tiles.append(cv2.resize(img, (int(W * 0.6), int(H * 0.6))))

    while len(tiles) % args.cols:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[i:i + args.cols]) for i in range(0, len(tiles), args.cols)])
    out = f"eye_check_{os.path.basename(os.path.normpath(args.root))}_{args.split}.png"
    cv2.imwrite(out, grid)
    print("wrote", out)


if __name__ == "__main__":
    main()
