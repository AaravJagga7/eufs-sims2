"""Helpers shared by all dataset converters.

Every converter writes this layout (Rule 4) into data_processed/<dataset>/ :
  images/<split>/<stem>.jpg      already letterboxed to 640x384
  labels_det/<split>/<stem>.txt  YOLO boxes (class cx cy w h), empty file = no objects
  splits.json                    {"train": [stems], "val": [...], "test": [...]}
Folders for tasks a dataset has no labels for are simply not created.
"""
import json
import os

import cv2
import numpy as np

from datasets.contract import IMG_H, IMG_W

IMG_EXT = (".jpg", ".jpeg", ".png")


def make_dirs(out_root, splits, kinds):
    for split in splits:
        for kind in kinds:
            os.makedirs(os.path.join(out_root, kind, split), exist_ok=True)


def read_yolo(path):
    """Read a YOLO label file. Returns (classes [N] int64, boxes [N, 4] float32).
    A missing or empty file gives empty arrays."""
    classes, boxes = [], []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                p = line.split()
                if len(p) >= 5:
                    classes.append(int(float(p[0])))
                    boxes.append([float(v) for v in p[1:5]])
    return np.array(classes, dtype=np.int64), np.array(boxes, dtype=np.float32).reshape(-1, 4)


def clean_boxes(classes, boxes, min_px=2):
    """Clip boxes to the image and drop boxes smaller than min_px in the 640x384 image."""
    if len(boxes) == 0:
        return classes, boxes
    xyxy = np.stack([boxes[:, 0] - boxes[:, 2] / 2, boxes[:, 1] - boxes[:, 3] / 2,
                     boxes[:, 0] + boxes[:, 2] / 2, boxes[:, 1] + boxes[:, 3] / 2], axis=1)
    xyxy = np.clip(xyxy, 0.0, 1.0)
    w, h = xyxy[:, 2] - xyxy[:, 0], xyxy[:, 3] - xyxy[:, 1]
    keep = (w * IMG_W >= min_px) & (h * IMG_H >= min_px)
    xyxy, classes = xyxy[keep], classes[keep]
    out = np.stack([(xyxy[:, 0] + xyxy[:, 2]) / 2, (xyxy[:, 1] + xyxy[:, 3]) / 2,
                    xyxy[:, 2] - xyxy[:, 0], xyxy[:, 3] - xyxy[:, 1]], axis=1)
    return classes, out.astype(np.float32)


def save_sample(out_root, split, stem, image, classes=None, boxes=None):
    """Write the image, and the label file if classes is not None (empty arrays = no objects)."""
    cv2.imwrite(os.path.join(out_root, "images", split, stem + ".jpg"), image,
                [cv2.IMWRITE_JPEG_QUALITY, 95])
    if classes is not None:
        with open(os.path.join(out_root, "labels_det", split, stem + ".txt"), "w") as f:
            for c, (cx, cy, w, h) in zip(classes, boxes):
                f.write(f"{int(c)} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")


def split_groups(groups, rng, fracs=(0.70, 0.15, 0.15)):
    """Shuffle whole groups (blocks, photos, videos...) and deal them into
    train / val / test. Items inside a group always stay together."""
    groups = list(groups)
    rng.shuffle(groups)
    n_train = round(len(groups) * fracs[0])
    n_val = round(len(groups) * fracs[1])
    return {"train": groups[:n_train],
            "val": groups[n_train:n_train + n_val],
            "test": groups[n_train + n_val:]}


def write_splits(out_root, splits):
    with open(os.path.join(out_root, "splits.json"), "w") as f:
        json.dump(splits, f, indent=1)
