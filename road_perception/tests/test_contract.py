"""Data check (owned by Person B).

Loads a few samples from every converted dataset and split and checks that
they follow the shared contract (datasets/contract.py).

Run:  pytest tests/test_contract.py -v
Set DATA_ROOT if your converted data is not in ./data_processed
"""
import os

import pytest
import torch

from datasets.contract import (IMG_H, IMG_W, NUM_DET_CLASSES, SEG_IGNORE,
                               DEPTH_MAX_M, check_sample, collate_fn)

DATA_ROOT = os.environ.get("DATA_ROOT", "data_processed")
N_SAMPLES = 4
SPLITS = ("train", "val", "test")


def _find_cases():
    """Every (dataset, split) pair that has images on disk."""
    cases = []
    if not os.path.isdir(DATA_ROOT):
        return cases
    for name in sorted(os.listdir(DATA_ROOT)):
        for split in SPLITS:
            if os.path.isdir(os.path.join(DATA_ROOT, name, "images", split)):
                cases.append((name, split))
    return cases


CASES = _find_cases()


def _load_dataset(name, split):
    # Imported here so the test file works before the loader is written.
    try:
        from datasets.multitask_dataset import MultiTaskDataset
    except ImportError:
        pytest.skip("MultiTaskDataset not written yet")
    return MultiTaskDataset(os.path.join(DATA_ROOT, name), split)


@pytest.mark.skipif(not CASES, reason=f"no converted data found in {DATA_ROOT}/")
@pytest.mark.parametrize("name,split", CASES)
def test_samples_follow_contract(name, split):
    ds = _load_dataset(name, split)
    assert len(ds) > 0, f"{name}/{split} is empty"

    for i in range(min(N_SAMPLES, len(ds))):
        s = ds[i]
        check_sample(s)  # shapes, dtypes, key names, 0-1 image

        # boxes: normalised YOLO format, valid classes
        if s["boxes"].numel():
            assert s["boxes"].min() >= 0.0 and s["boxes"].max() <= 1.0, "boxes outside 0-1"
            assert (s["boxes"][:, 2:] > 0).all(), "box with zero width or height"
            assert s["labels"].min() >= 0 and s["labels"].max() < NUM_DET_CLASSES

        # road mask: only 0, 1 or 255
        assert set(s["seg_mask"].unique().tolist()) <= {0, 1, SEG_IGNORE}

        # depth: metres, within range
        assert s["depth"].min() >= 0.0 and s["depth"].max() <= DEPTH_MAX_M + 1e-3

        # "no label" values must match the has_* flags
        if not s["has_det"]:
            assert s["boxes"].shape[0] == 0
        if not s["has_seg"]:
            assert (s["seg_mask"] == SEG_IGNORE).all()
        if not s["has_depth"]:
            assert (s["depth"] == 0).all()

        assert s["dataset"] and s["stem"]


@pytest.mark.skipif(not CASES, reason=f"no converted data found in {DATA_ROOT}/")
@pytest.mark.parametrize("name,split", CASES)
def test_batching_works(name, split):
    ds = _load_dataset(name, split)
    n = min(N_SAMPLES, len(ds))
    batch = collate_fn([ds[i] for i in range(n)])
    assert batch["img"].shape == (n, 3, IMG_H, IMG_W)
    assert batch["det_targets"].ndim == 2 and batch["det_targets"].shape[1] == 6
    assert batch["seg_mask"].shape == (n, IMG_H, IMG_W)
    assert batch["depth"].shape == (n, IMG_H, IMG_W)


def test_no_split_leakage():
    """Same file name must never appear in two splits of one dataset."""
    for name in sorted({c[0] for c in CASES}):
        seen = {}
        for split in SPLITS:
            folder = os.path.join(DATA_ROOT, name, "images", split)
            if not os.path.isdir(folder):
                continue
            for f in os.listdir(folder):
                stem = os.path.splitext(f)[0]
                assert stem not in seen, f"{name}: {stem} is in both {seen[stem]} and {split}"
                seen[stem] = split