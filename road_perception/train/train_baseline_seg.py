"""Road segmentation baseline: SegFormer-B0 trained on IDD + BDD road masks.

Saves to <project>/<run_name>/:
  last.pt      full training state, written every epoch (used to resume)
  best.pt      best model by validation mIoU, in the Rule 6 format
  history.csv  one line per epoch: train loss, val mIoU, val road IoU
Rerunning the same command resumes an unfinished run, and does nothing for a finished one.
"""
import argparse
import csv
import os
import random
import subprocess

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import ConcatDataset, DataLoader

from datasets.contract import SEG_IGNORE, collate_fn
from datasets.multitask_dataset import MultiTaskDataset
from datasets.transforms import MultitaskAugmentor
from models.segformer_baseline import SegFormerRoad
from train.config import load_config


class RoadAugment:
    """Person B's augmentor, plus a fix for a key-name mismatch: the augmentor returns
    the road mask as 'seg_mask' but MultiTaskDataset reads it back as 'mask'."""

    def __init__(self):
        self.aug = MultitaskAugmentor()

    def __call__(self, sample):
        out = self.aug(sample)
        if "seg_mask" in out:
            out["mask"] = out.pop("seg_mask")
        return out


def git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def make_loader(cfg, split, train):
    parts = [MultiTaskDataset(os.path.join(cfg["data_root"], name), split,
                              transform=RoadAugment() if train else None)
             for name in cfg["datasets"]]
    for ds in parts:
        if not ds.has_seg:
            raise ValueError(f"{ds.root} has no road masks for split '{split}'")
    return DataLoader(ConcatDataset(parts), batch_size=cfg["batch"], shuffle=train,
                      num_workers=cfg["workers"], collate_fn=collate_fn,
                      drop_last=train, pin_memory=torch.cuda.is_available())


@torch.no_grad()
def evaluate(model, loader, device):
    """mIoU over road / not-road, ignoring unlabelled pixels (same definition as eval_seg)."""
    model.eval()
    inter = torch.zeros(2, dtype=torch.float64)
    union = torch.zeros(2, dtype=torch.float64)
    for batch in loader:
        gt = batch["seg_mask"].to(device)
        with torch.autocast(device_type=device.type, enabled=device.type == "cuda"):
            pred = model(batch["img"].to(device)).argmax(1)
        valid = gt != SEG_IGNORE
        for c in (0, 1):
            p, g = (pred == c) & valid, (gt == c) & valid
            inter[c] += (p & g).sum().item()
            union[c] += (p | g).sum().item()
    iou = inter / union.clamp(min=1)
    return {"mIoU": iou.mean().item(), "IoU_road": iou[1].item()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--project", default=None, help="where to save, e.g. a Drive folder")
    p.add_argument("--data-root", default=None)
    p.add_argument("--epochs", type=int, default=None)
    p.add_argument("--batch", type=int, default=None)
    args = p.parse_args()
    cfg = load_config(args.config, project=args.project, data_root=args.data_root,
                      epochs=args.epochs, batch=args.batch)

    run_dir = os.path.join(os.path.abspath(cfg["project"]), cfg["run_name"])
    os.makedirs(run_dir, exist_ok=True)
    last_path, best_path = os.path.join(run_dir, "last.pt"), os.path.join(run_dir, "best.pt")
    hist_path = os.path.join(run_dir, "history.csv")

    random.seed(cfg["seed"])
    np.random.seed(cfg["seed"])
    torch.manual_seed(cfg["seed"])
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    train_loader = make_loader(cfg, "train", train=True)
    val_loader = make_loader(cfg, "val", train=False)
    print(f"Train images: {len(train_loader.dataset)}, val images: {len(val_loader.dataset)}")

    model = SegFormerRoad(cfg["model"], pretrained=True).to(device)
    opt = torch.optim.AdamW(model.param_groups(cfg["lr"], cfg["head_lr_mult"]),
                            weight_decay=cfg["weight_decay"])
    total_steps = cfg["epochs"] * len(train_loader)
    sched = torch.optim.lr_scheduler.LambdaLR(opt, lambda s: (1 - s / total_steps) ** 1.0)
    scaler = torch.amp.GradScaler(enabled=device.type == "cuda")

    start_epoch, best_miou = 0, -1.0
    if os.path.exists(last_path):
        state = torch.load(last_path, map_location="cpu", weights_only=False)
        if state["epoch"] + 1 >= cfg["epochs"]:
            print(f"Run already finished: {run_dir}. Nothing to do.")
            return
        model.load_state_dict(state["model"])
        opt.load_state_dict(state["optimizer"])
        sched.load_state_dict(state["scheduler"])
        scaler.load_state_dict(state["scaler"])
        start_epoch, best_miou = state["epoch"] + 1, state["best_miou"]
        print(f"Found unfinished run. Resuming from epoch {start_epoch + 1}")
    else:
        print(f"Starting new run: {cfg['run_name']}")

    for epoch in range(start_epoch, cfg["epochs"]):
        model.train()
        loss_sum, n = 0.0, 0
        for batch in train_loader:
            img = batch["img"].to(device, non_blocking=True)
            gt = batch["seg_mask"].to(device, non_blocking=True)
            with torch.autocast(device_type=device.type, enabled=device.type == "cuda"):
                loss = F.cross_entropy(model(img), gt, ignore_index=SEG_IGNORE)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            loss_sum += loss.item()
            n += 1

        metrics = evaluate(model, val_loader, device)
        train_loss = loss_sum / max(n, 1)
        print(f"Epoch {epoch + 1}/{cfg['epochs']}  loss {train_loss:.4f}  "
              f"val mIoU {metrics['mIoU']:.4f}  road IoU {metrics['IoU_road']:.4f}")

        new_file = not os.path.exists(hist_path)
        with open(hist_path, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(["epoch", "train_loss", "val_mIoU", "val_IoU_road"])
            w.writerow([epoch + 1, f"{train_loss:.5f}", f"{metrics['mIoU']:.5f}",
                        f"{metrics['IoU_road']:.5f}"])

        if metrics["mIoU"] > best_miou:
            best_miou = metrics["mIoU"]
            torch.save({"model": model.state_dict(), "config": cfg, "epoch": epoch + 1,
                        "metrics": metrics, "git_commit": git_commit()}, best_path)
            print(f"  New best model saved: {best_path}")

        # Write to a temp file first, so a disconnect mid-save can't corrupt last.pt
        torch.save({"model": model.state_dict(), "optimizer": opt.state_dict(),
                    "scheduler": sched.state_dict(), "scaler": scaler.state_dict(),
                    "epoch": epoch, "best_miou": best_miou}, last_path + ".tmp")
        os.replace(last_path + ".tmp", last_path)

    print(f"Done. Best val mIoU {best_miou:.4f}. Saved {best_path}")


if __name__ == "__main__":
    main()
