"""MultiTaskDataset: reads one converted dataset folder and returns samples in the
agreed format (Rule 4 / datasets/contract.py).

  ds = MultiTaskDataset("data_processed/rdd", "train")
  sample = ds[0]     # dict with image, boxes, labels, seg_mask, depth, has_*, dataset, stem

A task whose folder does not exist for this dataset (labels_det / masks / depth)
gets the "no label" values and has_* = False. If the folder exists but one file is
missing, that is a converter bug and an error is raised.
"""
import json
import os

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from datasets.contract import DEPTH_PNG_SCALE, empty_labels
from datasets.converters.common import read_yolo


def load_arrays(root, split, stem, has_det, has_seg, has_depth):
    """Read one sample from disk as numpy arrays (None for tasks without labels)."""
    img = cv2.imread(os.path.join(root, "images", split, stem + ".jpg"))
    if img is None:
        raise FileNotFoundError(f"cannot read image {stem} in {root}/images/{split}")
    out = {"image": cv2.cvtColor(img, cv2.COLOR_BGR2RGB),
           "classes": None, "boxes": None, "mask": None, "depth": None}

    if has_det:
        path = os.path.join(root, "labels_det", split, stem + ".txt")
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        out["classes"], out["boxes"] = read_yolo(path)
    if has_seg:
        path = os.path.join(root, "masks", split, stem + ".png")
        m = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if m is None:
            raise FileNotFoundError(path)
        out["mask"] = m
    if has_depth:
        path = os.path.join(root, "depth", split, stem + ".png")
        d = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if d is None:
            raise FileNotFoundError(path)
        out["depth"] = d.astype(np.float32) / DEPTH_PNG_SCALE   # PNG holds metres x 256
    return out


class MultiTaskDataset(Dataset):
    def __init__(self, root, split, transform=None):
        self.root, self.split = root, split
        self.transform = transform
        self.name = os.path.basename(os.path.normpath(root))
        with open(os.path.join(root, "splits.json")) as f:
            self.stems = json.load(f)[split]
        self.has_det = os.path.isdir(os.path.join(root, "labels_det", split))
        self.has_seg = os.path.isdir(os.path.join(root, "masks", split))
        self.has_depth = os.path.isdir(os.path.join(root, "depth", split))

    def __len__(self):
        return len(self.stems)

    def __getitem__(self, i):
        stem = self.stems[i]
        a = load_arrays(self.root, self.split, stem, self.has_det, self.has_seg, self.has_depth)
        if self.transform is not None:
            a = self.transform(a)

        sample = empty_labels()   # boxes, labels, seg_mask, depth with "no label" values
        sample["image"] = torch.from_numpy(a["image"]).permute(2, 0, 1).contiguous().float() / 255.0
        if self.has_det:
            boxes = a["boxes"] if a["boxes"] is not None else np.zeros((0, 4), dtype=np.float32)
            classes = a["classes"] if a["classes"] is not None else np.zeros((0,), dtype=np.int64)
            sample["boxes"] = torch.from_numpy(boxes).float()
            sample["labels"] = torch.from_numpy(classes).long()
        if self.has_seg:
            sample["seg_mask"] = torch.from_numpy(a["mask"].astype(np.int64))
        if self.has_depth:
            sample["depth"] = torch.from_numpy(a["depth"].astype(np.float32))
        sample.update(has_det=self.has_det, has_seg=self.has_seg, has_depth=self.has_depth,
                      dataset=self.name, stem=stem)
        return sample
