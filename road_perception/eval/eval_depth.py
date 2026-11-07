"""Depth evaluation (owned by Person B).

Computes metric depth errors on ground truth depth:
- depth_rmse_0_10:  RMSE for depths in [0, 10) metres
- depth_rmse_10_25: RMSE for depths in [10, 25) metres
- depth_rmse_25_50: RMSE for depths in [25, 50) metres
- depth_absrel:     mean |pred - gt| / gt across all valid depths

Run standalone:
  python -m eval.eval_depth --dataset kitti --split test --model stereosgbm
"""
import argparse
import numpy as np
import torch
from torch.utils.data import DataLoader

from datasets.contract import collate_fn
from datasets.multitask_dataset import MultiTaskDataset


def compute_depth_metrics(preds, gts):
    """Computes fixed depth metrics between predicted and ground-truth depth maps.
    preds: array or tensor of shape (N, H, W) in metres
    gts:   array or tensor of shape (N, H, W) in metres
    """
    if isinstance(preds, torch.Tensor):
        preds = preds.detach().cpu().numpy()
    if isinstance(gts, torch.Tensor):
        gts = gts.detach().cpu().numpy()

    # Valid mask: GT must have positive depth, pred must have positive depth
    valid = (gts > 0.0) & (preds > 0.0) & np.isfinite(gts) & np.isfinite(preds)

    def band_rmse(d_min, d_max):
        band = valid & (gts >= d_min) & (gts < d_max)
        if not np.any(band):
            return 0.0
        diff = preds[band] - gts[band]
        return float(np.sqrt(np.mean(diff ** 2)))

    rmse_0_10 = band_rmse(0.0, 10.0)
    rmse_10_25 = band_rmse(10.0, 25.0)
    rmse_25_50 = band_rmse(25.0, 50.0)

    if np.any(valid):
        absrel = float(np.mean(np.abs(preds[valid] - gts[valid]) / gts[valid]))
    else:
        absrel = 0.0

    return {
        "depth_rmse_0_10": rmse_0_10,
        "depth_rmse_10_25": rmse_10_25,
        "depth_rmse_25_50": rmse_25_50,
        "depth_absrel": absrel,
    }


def evaluate_depth(model, data_root="data_processed/kitti", split="test", batch_size=4):
    """Evaluates any model with predict(images) on the depth task."""
    if hasattr(model, "reset"):
        model.reset()

    ds = MultiTaskDataset(data_root, split)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=False, collate_fn=collate_fn)

    all_preds = []
    all_gts = []

    for batch in loader:
        gts = batch["depth"].numpy()
        imgs = batch["img"]
        stems = batch.get("stem")
        # Model returns dict with 'depth'
        try:
            res = model.predict(imgs, stems=stems)
        except TypeError:
            res = model.predict(imgs)
        pred_depth = res.get("depth")
        if pred_depth is None:
            return {}
        if isinstance(pred_depth, torch.Tensor):
            pred_depth = pred_depth.detach().cpu().numpy()
        all_preds.append(pred_depth)
        all_gts.append(gts)

    if not all_preds:
        return {}
    all_preds = np.concatenate(all_preds, axis=0)
    all_gts = np.concatenate(all_gts, axis=0)
    return compute_depth_metrics(all_preds, all_gts)


if __name__ == "__main__":
    from eval.depth_wrappers import get_depth_model

    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data_processed/kitti")
    parser.add_argument("--split", default="test")
    parser.add_argument("--model", default="stereosgbm", choices=["stereosgbm", "depthanything"])
    args = parser.parse_args()

    model = get_depth_model(args.model, dataset_root=args.dataset, split=args.split)
    metrics = evaluate_depth(model, data_root=args.dataset, split=args.split)
    print("Depth evaluation results:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")
