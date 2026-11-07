"""Shared rules between Person A (models) and Person B (data).
Change only when both agree."""
import torch

IMG_H, IMG_W = 384, 640
NUM_DET_CLASSES = 8
NUM_SEG_CLASSES = 2
SEG_IGNORE = 255
DEPTH_MAX_M = 80.0
DEPTH_PNG_SCALE = 256.0   # depth PNG value = metres * 256
SEED = 42

SAMPLE_KEYS = ("image", "boxes", "labels", "seg_mask", "depth",
               "has_det", "has_seg", "has_depth", "dataset", "stem")


def empty_labels():
    """Values used when a dataset has no label for a task."""
    return {
        "boxes": torch.zeros((0, 4), dtype=torch.float32),
        "labels": torch.zeros((0,), dtype=torch.int64),
        "seg_mask": torch.full((IMG_H, IMG_W), SEG_IGNORE, dtype=torch.int64),
        "depth": torch.zeros((IMG_H, IMG_W), dtype=torch.float32),
    }


def check_sample(s):
    """Raise an error if a sample breaks the rules."""
    missing = set(SAMPLE_KEYS) - set(s.keys())
    assert not missing, f"missing keys: {missing}"
    assert s["image"].shape == (3, IMG_H, IMG_W) and s["image"].dtype == torch.float32
    assert 0.0 <= s["image"].min() and s["image"].max() <= 1.0
    assert s["boxes"].ndim == 2 and s["boxes"].shape[1] == 4
    assert s["labels"].shape[0] == s["boxes"].shape[0]
    assert s["seg_mask"].shape == (IMG_H, IMG_W) and s["seg_mask"].dtype == torch.int64
    assert s["depth"].shape == (IMG_H, IMG_W) and s["depth"].dtype == torch.float32
    for k in ("has_det", "has_seg", "has_depth"):
        assert isinstance(s[k], bool), f"{k} must be True/False"


def collate_fn(samples):
    """Group samples into a batch. Boxes become [M, 6]:
    batch_idx, cls, cx, cy, w, h (the format the YOLO loss expects)."""
    targets = []
    for i, s in enumerate(samples):
        n = s["boxes"].shape[0]
        if n:
            idx = torch.full((n, 1), float(i))
            cls = s["labels"].float().unsqueeze(1)
            targets.append(torch.cat([idx, cls, s["boxes"]], dim=1))
    return {
        "img": torch.stack([s["image"] for s in samples]),
        "det_targets": torch.cat(targets) if targets else torch.zeros((0, 6)),
        "seg_mask": torch.stack([s["seg_mask"] for s in samples]),
        "depth": torch.stack([s["depth"] for s in samples]),
        "has_det": torch.tensor([s["has_det"] for s in samples]),
        "has_seg": torch.tensor([s["has_seg"] for s in samples]),
        "has_depth": torch.tensor([s["has_depth"] for s in samples]),
        "dataset": [s["dataset"] for s in samples],
        "stem": [s["stem"] for s in samples],
    }