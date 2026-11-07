"""Train the combined model (detection + road + distance) across all datasets.

Each batch comes from one dataset (Person B's MultitaskBatchSampler), and each task's
loss uses only the images that have that label. Saves to <project>/<run_name>/:
  last.pt      full training state, written every epoch (used to resume)
  best.pt      best model by validation score, in the Rule 6 format
  history.csv  per-epoch losses, learned task weights and validation metrics
Rerunning the same command resumes an unfinished run, and does nothing for a finished one.

Comparison versions (Stage 3) only change settings, e.g.:
  --run-name <name> --loss-weighting equal
  --run-name <name> --tasks det seg
"""
import argparse
import csv
import math
import os
import random
import subprocess

import numpy as np
import torch
from torch.utils.data import DataLoader

from datasets.contract import DEPTH_MAX_M, IMG_H, IMG_W, NUM_DET_CLASSES, SEG_IGNORE, collate_fn
from datasets.multitask_dataset import MultiTaskDataset
from datasets.samplers import ConcatMultiTaskDataset
from eval.eval_det import compute_class_ap
from losses.multitask_loss import TASKS, MultiTaskLoss
from models.multitask import MultiTaskNet
from train.config import load_config
from train.train_baseline_seg import RoadAugment   # augmentor + fix for the mask key mismatch


def git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def to_device(batch, device):
    return {k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v)
            for k, v in batch.items()}


@torch.no_grad()
def evaluate(model, loaders, device, conf=0.001, iou=0.7):
    """mAP50 (detection), mIoU (road) and AbsRel (distance) on the validation splits."""
    from ultralytics.utils.nms import non_max_suppression

    model.eval()
    dets, gts = {}, {}
    inter, union = torch.zeros(2, dtype=torch.float64), torch.zeros(2, dtype=torch.float64)
    rel_sum, rel_n = 0.0, 0
    img_id = 0
    for loader in loaders.values():
        for batch in loader:
            batch = to_device(batch, device)
            with torch.autocast(device_type=device.type, enabled=device.type == "cuda"):
                out = model(batch["img"])
            b = batch["img"].shape[0]
            if out["det"] is not None and batch["has_det"].any():
                preds = non_max_suppression(out["det"][0].float(), conf_thres=conf, iou_thres=iou)
                t = batch["det_targets"].cpu().numpy()
                for i in range(b):
                    if not batch["has_det"][i]:
                        continue
                    for row in t[t[:, 0] == i]:
                        cx, cy, w, h = row[2:]
                        gts.setdefault(int(row[1]), {}).setdefault(img_id + i, []).append(
                            [(cx - w / 2) * IMG_W, (cy - h / 2) * IMG_H,
                             (cx + w / 2) * IMG_W, (cy + h / 2) * IMG_H])
                    for x1, y1, x2, y2, s, c in preds[i][:, :6].cpu().numpy():
                        dets.setdefault(int(c), []).append((x1, y1, x2, y2, s, img_id + i))
            if out["seg"] is not None and batch["has_seg"].any():
                keep = batch["has_seg"]
                pred, gt = out["seg"][keep].argmax(1), batch["seg_mask"][keep]
                valid = gt != SEG_IGNORE
                for c in (0, 1):
                    p, g = (pred == c) & valid, (gt == c) & valid
                    inter[c] += (p & g).sum().item()
                    union[c] += (p | g).sum().item()
            if out["depth"] is not None and batch["has_depth"].any():
                keep = batch["has_depth"]
                pred, gt = out["depth"][keep, 0].float(), batch["depth"][keep]
                valid = (gt > 0) & (gt <= DEPTH_MAX_M)
                rel_sum += ((pred[valid] - gt[valid]).abs() / gt[valid]).sum().item()
                rel_n += int(valid.sum().item())
            img_id += b

    metrics = {}
    if gts:
        aps = [compute_class_ap(dets.get(c, []), gts[c]) for c in range(NUM_DET_CLASSES) if c in gts]
        metrics["mAP50"] = float(np.nanmean(aps))
    if union.sum() > 0:
        metrics["mIoU"] = (inter / union.clamp(min=1)).mean().item()
    if rel_n > 0:
        metrics["depth_absrel"] = rel_sum / rel_n
    parts = [metrics.get("mAP50"), metrics.get("mIoU")]
    if "depth_absrel" in metrics:
        parts.append(max(0.0, 1.0 - metrics["depth_absrel"]))
    parts = [p for p in parts if p is not None]
    metrics["score"] = float(np.mean(parts)) if parts else 0.0
    return metrics


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--project", default=None, help="where to save, e.g. a Drive folder")
    p.add_argument("--data-root", default=None)
    p.add_argument("--run-name", default=None)
    p.add_argument("--tasks", nargs="+", default=None, choices=TASKS)
    p.add_argument("--loss-weighting", default=None, choices=["uncertainty", "equal"])
    p.add_argument("--epochs", type=int, default=None)
    p.add_argument("--batch", type=int, default=None)
    p.add_argument("--workers", type=int, default=None)
    p.add_argument("--limit-batches", type=int, default=None,
                   help="train on only this many batches per epoch (quick tests)")
    p.add_argument("--allow-missing-depth", action="store_true",
                   help="start even if no training dataset has distance labels yet")
    args = p.parse_args()
    cfg = load_config(args.config, project=args.project, data_root=args.data_root,
                      run_name=args.run_name, tasks=args.tasks,
                      loss_weighting=args.loss_weighting, epochs=args.epochs, batch=args.batch,
                      workers=args.workers)

    run_dir = os.path.join(os.path.abspath(cfg["project"]), cfg["run_name"])
    os.makedirs(run_dir, exist_ok=True)
    last_path, best_path = os.path.join(run_dir, "last.pt"), os.path.join(run_dir, "best.pt")
    hist_path = os.path.join(run_dir, "history.csv")

    random.seed(cfg["seed"])
    np.random.seed(cfg["seed"])
    torch.manual_seed(cfg["seed"])
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Data: one MultiTaskDataset per dataset; each batch comes from a single dataset
    train_sets = {n: MultiTaskDataset(os.path.join(cfg["data_root"], n), "train",
                                      transform=RoadAugment()) for n in cfg["datasets"]}
    if "depth" in cfg["tasks"] and not any(ds.has_depth for ds in train_sets.values()):
        if not args.allow_missing_depth:
            raise SystemExit("No training dataset has distance labels (depth/ folder) yet. "
                             "Wait for Person B's Depth Anything labels before full training.")
        print("WARNING: no distance labels found; the distance head will not learn in this run.")
    concat = ConcatMultiTaskDataset(train_sets)
    train_loader = DataLoader(concat, batch_sampler=concat.make_sampler(cfg["batch"]),
                              num_workers=cfg["workers"], collate_fn=collate_fn,
                              pin_memory=device.type == "cuda")
    val_loaders = {n: DataLoader(MultiTaskDataset(os.path.join(cfg["data_root"], n), "val"),
                                 batch_size=cfg["batch"], num_workers=cfg["workers"],
                                 collate_fn=collate_fn)
                   for n in cfg["datasets"]}
    for n, ds in train_sets.items():
        print(f"  {n}: {len(ds)} train images | det={ds.has_det} seg={ds.has_seg} depth={ds.has_depth}")

    steps_per_epoch = len(train_loader)
    if args.limit_batches:
        steps_per_epoch = min(steps_per_epoch, args.limit_batches)

    model = MultiTaskNet(weights=cfg["pretrained"], pretrained=True, tasks=cfg["tasks"]).to(device)
    criterion = MultiTaskLoss(model, weighting=cfg["loss_weighting"]).to(device)
    groups = model.param_groups(cfg["lr_body"], cfg["lr_heads"], cfg["weight_decay"])
    groups.append({"params": [criterion.log_vars], "lr": cfg["lr_log_vars"], "weight_decay": 0.0})
    opt = torch.optim.AdamW(groups)

    total_steps = cfg["epochs"] * steps_per_epoch
    warmup = cfg["warmup_epochs"] * steps_per_epoch

    def lr_factor(step):
        if step < warmup:
            return 0.1 + 0.9 * step / max(1, warmup)
        t = (step - warmup) / max(1, total_steps - warmup)
        return 0.01 + 0.99 * 0.5 * (1 + math.cos(math.pi * min(1.0, t)))

    sched = torch.optim.lr_scheduler.LambdaLR(opt, lr_factor)
    scaler = torch.amp.GradScaler(enabled=device.type == "cuda")

    start_epoch, best_score = 0, -1.0
    if os.path.exists(last_path):
        state = torch.load(last_path, map_location="cpu", weights_only=False)
        if state["epoch"] + 1 >= cfg["epochs"]:
            print(f"Run already finished: {run_dir}. Nothing to do.")
            return
        model.load_state_dict(state["model"])
        criterion.load_state_dict(state["criterion"])
        opt.load_state_dict(state["optimizer"])
        sched.load_state_dict(state["scheduler"])
        scaler.load_state_dict(state["scaler"])
        start_epoch, best_score = state["epoch"] + 1, state["best_score"]
        print(f"Found unfinished run. Resuming from epoch {start_epoch + 1}")
    else:
        print(f"Starting new run: {cfg['run_name']}  tasks={cfg['tasks']}  "
              f"weighting={cfg['loss_weighting']}")

    for epoch in range(start_epoch, cfg["epochs"]):
        model.train()
        sums = {t: 0.0 for t in TASKS}
        counts = {t: 0 for t in TASKS}
        for step, batch in enumerate(train_loader):
            if step >= steps_per_epoch:
                break
            batch = to_device(batch, device)
            with torch.autocast(device_type=device.type, enabled=device.type == "cuda"):
                out = model(batch["img"])
            loss, parts = criterion(out, batch)
            if not loss.requires_grad:      # nothing to learn from this batch
                sched.step()
                continue
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.unscale_(opt)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            scaler.step(opt)
            scaler.update()
            sched.step()
            for t, v in parts.items():
                if v is not None:
                    sums[t] += v
                    counts[t] += 1

        avg = {t: (sums[t] / counts[t] if counts[t] else None) for t in TASKS}
        sigma = {t: float(torch.exp(0.5 * criterion.log_vars[i]).item()) for i, t in enumerate(TASKS)}
        metrics = evaluate(model, val_loaders, device)
        fmt = lambda v: "-" if v is None else f"{v:.4f}"
        print(f"Epoch {epoch + 1}/{cfg['epochs']}  loss det {fmt(avg['det'])} "
              f"seg {fmt(avg['seg'])} depth {fmt(avg['depth'])} | "
              f"val mAP50 {fmt(metrics.get('mAP50'))} mIoU {fmt(metrics.get('mIoU'))} "
              f"AbsRel {fmt(metrics.get('depth_absrel'))} score {metrics['score']:.4f}")

        row = {"epoch": epoch + 1,
               **{f"loss_{t}": fmt(avg[t]) for t in TASKS},
               **{f"sigma_{t}": f"{sigma[t]:.4f}" for t in TASKS},
               "lr_body": f"{opt.param_groups[0]['lr']:.2e}",
               **{k: f"{metrics[k]:.5f}" if k in metrics else "-"
                  for k in ("mAP50", "mIoU", "depth_absrel", "score")}}
        new_file = not os.path.exists(hist_path)
        with open(hist_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                w.writeheader()
            w.writerow(row)

        if metrics["score"] > best_score:
            best_score = metrics["score"]
            torch.save({"model": model.state_dict(), "config": cfg, "epoch": epoch + 1,
                        "metrics": metrics, "git_commit": git_commit()}, best_path)
            print(f"  New best model saved: {best_path}")

        # Write to a temp file first, so a disconnect mid-save can't corrupt last.pt
        torch.save({"model": model.state_dict(), "criterion": criterion.state_dict(),
                    "optimizer": opt.state_dict(), "scheduler": sched.state_dict(),
                    "scaler": scaler.state_dict(), "epoch": epoch, "best_score": best_score},
                   last_path + ".tmp")
        os.replace(last_path + ".tmp", last_path)

    print(f"Done. Best val score {best_score:.4f}. Saved {best_path}")


if __name__ == "__main__":
    main()
