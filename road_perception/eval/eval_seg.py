"""Segmentation evaluation (owned by Person B).

Computes road segmentation accuracy ignoring unlabelled pixels (SEG_IGNORE = 255):
- mIoU:     mean Intersection-over-Union across road (1) and non-road (0)
- IoU_road: Intersection-over-Union specifically for the road class

Run standalone:
  python -m eval.eval_seg --dataset data_processed/idd --split test
"""
import argparse
import numpy as np
import torch
from torch.utils.data import DataLoader

from datasets.contract import SEG_IGNORE, collate_fn
from datasets.multitask_dataset import MultiTaskDataset


def compute_seg_metrics(preds, gts):
    """Computes segmentation IoU metrics.
    preds: array or tensor of shape (N, H, W) with binary values 0 or 1
    gts:   array or tensor of shape (N, H, W) with values 0, 1, or 255
    """
    if isinstance(preds, torch.Tensor):
        preds = preds.detach().cpu().numpy()
    if isinstance(gts, torch.Tensor):
        gts = gts.detach().cpu().numpy()

    valid = (gts != SEG_IGNORE)

    # Road (class 1)
    p_road = (preds == 1) & valid
    g_road = (gts == 1) & valid
    inter_road = np.logical_and(p_road, g_road).sum()
    union_road = np.logical_or(p_road, g_road).sum()
    iou_road = float(inter_road / max(1, union_road))

    # Non-road (class 0)
    p_nonroad = (preds == 0) & valid
    g_nonroad = (gts == 0) & valid
    inter_nonroad = np.logical_and(p_nonroad, g_nonroad).sum()
    union_nonroad = np.logical_or(p_nonroad, g_nonroad).sum()
    iou_nonroad = float(inter_nonroad / max(1, union_nonroad))

    miou = float((iou_road + iou_nonroad) / 2.0)

    return {
        "mIoU": miou,
        "IoU_road": iou_road,
    }


def evaluate_seg(model, data_root="data_processed/idd", split="test", batch_size=4):
    """Evaluates any model with predict(images) on the segmentation task."""
    ds = MultiTaskDataset(data_root, split)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=False, collate_fn=collate_fn)

    all_preds = []
    all_gts = []

    for batch in loader:
        gts = batch["seg_mask"].numpy()
        imgs = batch["img"]
        res = model.predict(imgs)
        pred_seg = res.get("seg")
        if pred_seg is None:
            return {}
        if isinstance(pred_seg, torch.Tensor):
            pred_seg = pred_seg.detach().cpu().numpy()
        all_preds.append(pred_seg)
        all_gts.append(gts)

    if not all_preds:
        return {}
    all_preds = np.concatenate(all_preds, axis=0)
    all_gts = np.concatenate(all_gts, axis=0)
    return compute_seg_metrics(all_preds, all_gts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data_processed/idd")
    parser.add_argument("--split", default="test")
    args = parser.parse_args()

    # Dummy baseline predictor for testing
    class DummyRoadPredictor:
        def predict(self, imgs):
            b = imgs.shape[0]
            return {"boxes": None, "seg": np.ones((b, 384, 640), dtype=np.uint8), "depth": None}

    model = DummyRoadPredictor()
    metrics = evaluate_seg(model, data_root=args.dataset, split=args.split)
    print("Segmentation evaluation results:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")
