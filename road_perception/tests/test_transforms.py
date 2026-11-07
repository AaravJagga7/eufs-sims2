"""Tests for datasets/transforms.py (owned by Person B)."""
import numpy as np
import pytest

from datasets.contract import IMG_H, IMG_W
from datasets.transforms import letterbox


@pytest.mark.parametrize("h,w", [(720, 1280), (600, 600), (375, 1242), (1080, 1920), (300, 200)])
def test_output_size(h, w):
    out = letterbox(np.zeros((h, w, 3), np.uint8))
    assert out["image"].shape == (IMG_H, IMG_W, 3)


def test_box_moves_with_image():
    """A white square at a known place; the transformed box must sit on it."""
    img = np.zeros((600, 600, 3), np.uint8)
    img[300:400, 100:200] = 255                      # x 100-200, y 300-400
    box = [[150 / 600, 350 / 600, 100 / 600, 100 / 600]]
    out = letterbox(img, boxes=box)

    ys, xs = np.where(out["image"][..., 0] > 127)
    cx, cy = (xs.min() + xs.max() + 1) / 2, (ys.min() + ys.max() + 1) / 2
    w, h = xs.max() + 1 - xs.min(), ys.max() + 1 - ys.min()
    bx, by, bw, bh = out["boxes"][0] * [IMG_W, IMG_H, IMG_W, IMG_H]
    assert abs(bx - cx) <= 1.5 and abs(by - cy) <= 1.5
    assert abs(bw - w) <= 2 and abs(bh - h) <= 2


def test_not_stretched():
    """A square object must stay square after letterboxing a wide image."""
    img = np.zeros((375, 1242, 3), np.uint8)
    box = [[0.5, 0.5, 100 / 1242, 100 / 375 * (375 / 375)]]   # 100 px wide, ~100 px tall
    box[0][3] = 100 / 375
    out = letterbox(img, boxes=box)
    _, _, bw, bh = out["boxes"][0] * [IMG_W, IMG_H, IMG_W, IMG_H]
    assert abs(bw - bh) <= 1.0


def test_mask_and_depth_padding_and_values():
    h = w = 600
    mask = np.random.RandomState(0).randint(0, 2, (h, w)).astype(np.uint8)
    depth = (np.random.RandomState(1).rand(h, w) * 80).astype(np.float32)
    out = letterbox(np.zeros((h, w, 3), np.uint8), seg_mask=mask, depth=depth)

    # scale 0.64 -> content is 384x384, padded 128 left and right
    assert (out["seg_mask"][:, :128] == 255).all() and (out["seg_mask"][:, 512:] == 255).all()
    assert (out["depth"][:, :128] == 0).all() and (out["depth"][:, 512:] == 0).all()
    # no new values invented by resizing
    assert set(np.unique(out["seg_mask"][:, 128:512])) <= {0, 1}
    assert np.isin(out["depth"][:, 128:512], depth).all()
    assert out["seg_mask"].dtype == np.uint8 and out["depth"].dtype == np.float32


def test_returns_scale_and_pad():
    out = letterbox(np.zeros((600, 600, 3), np.uint8))
    assert out["scale"] == pytest.approx(0.64)
    assert out["pad"] == (128, 0)


def test_augment_horizontal_flip():
    """Horizontal flip must mirror boxes, mask, and depth exactly."""
    from datasets.transforms import augment_multitask
    import random

    img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    boxes = np.array([[0.25, 0.5, 0.1, 0.2]], dtype=np.float32)
    mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    mask[:, :IMG_W // 2] = 1
    depth = np.zeros((IMG_H, IMG_W), dtype=np.float32)
    depth[:, :IMG_W // 2] = 15.0

    aug = augment_multitask(img, boxes=boxes, seg_mask=mask, depth=depth,
                            p_flip=1.0, zoom_range=(1.0, 1.0), p_color=0.0, p_blur=0.0)

    # Box cx should flip from 0.25 to 0.75
    assert abs(aug["boxes"][0, 0] - 0.75) < 1e-4
    # Mask should now have 1s on right half
    assert (aug["seg_mask"][:, IMG_W // 2:] == 1).all()
    # Depth should now have 15.0 on right half
    assert (aug["depth"][:, IMG_W // 2:] == 15.0).all()


def test_augment_with_dataset():
    """Verify MultiTaskDataset works with MultitaskAugmentor and follows contract.
    Skipped automatically on machines where data_processed/speedbump is absent."""
    import os
    if not os.path.exists("data_processed/speedbump/splits.json"):
        pytest.skip("data_processed/speedbump not found – run the speedbump converter first")

    from datasets.contract import check_sample
    from datasets.multitask_dataset import MultiTaskDataset
    from datasets.transforms import MultitaskAugmentor

    ds = MultiTaskDataset("data_processed/speedbump", "train", transform=MultitaskAugmentor())
    assert len(ds) > 0
    for i in range(min(5, len(ds))):
        s = ds[i]
        check_sample(s)
