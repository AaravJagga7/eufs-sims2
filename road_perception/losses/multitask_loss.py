"""Combined loss for the multi-task model.

Each task's loss uses only the images that have labels for it (has_det / has_seg /
has_depth). A task with no labelled images in the batch is skipped, so it adds
nothing and its head gets no gradient from that batch.

Weighting between tasks (config "loss_weighting"):
  "uncertainty": learned weights (Kendall et al., 2018):
                   L = sum_t  0.5 * exp(-s_t) * L_t + 0.5 * s_t,   s_t = log(sigma_t^2), learned
  "equal":       L = sum_t L_t   (comparison version for the ablation)
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
from ultralytics.utils.loss import v8DetectionLoss

from datasets.contract import SEG_IGNORE

TASKS = ("det", "seg", "depth")


def seg_loss(logits, gt):
    """Cross-entropy + 0.5 x Dice on the road class, ignoring unlabelled pixels."""
    valid = gt != SEG_IGNORE
    if not valid.any():
        return None
    logits = logits.float()
    ce = F.cross_entropy(logits, gt, ignore_index=SEG_IGNORE)
    road_prob = logits.softmax(1)[:, 1][valid]
    road_gt = (gt[valid] == 1).float()
    dice = 1 - (2 * (road_prob * road_gt).sum() + 1) / (road_prob.sum() + road_gt.sum() + 1)
    return ce + 0.5 * dice


def silog_loss(pred, gt, lam=0.85):
    """Scale-invariant log loss on pixels with a known distance (gt > 0)."""
    valid = gt > 0
    if not valid.any():
        return None
    d = torch.log(pred.float()[valid].clamp(min=1e-3)) - torch.log(gt[valid])
    return torch.sqrt((d ** 2).mean() - lam * d.mean() ** 2 + 1e-8)


class MultiTaskLoss(nn.Module):
    def __init__(self, model, weighting="uncertainty"):
        super().__init__()
        self.weighting = weighting
        self.tasks = model.tasks
        self.det_loss = v8DetectionLoss(model.yolo)
        self.log_vars = nn.Parameter(torch.zeros(len(TASKS)))   # s_t, one per task

    def _det(self, det_out, batch, keep):
        idx = keep.nonzero().flatten()
        if len(idx) == 0:
            return None
        preds = {"boxes": det_out["boxes"][idx], "scores": det_out["scores"][idx],
                 "feats": [f[idx] for f in det_out["feats"]]}
        t = batch["det_targets"].to(idx.device)
        new_pos = torch.full((len(keep),), -1, device=idx.device, dtype=torch.long)
        new_pos[idx] = torch.arange(len(idx), device=idx.device)
        t = t[new_pos[t[:, 0].long()] >= 0]
        targets = {"batch_idx": new_pos[t[:, 0].long()].float(), "cls": t[:, 1], "bboxes": t[:, 2:]}
        return self.det_loss.get_assigned_targets_and_loss(preds, targets)[1].sum()

    def forward(self, out, batch):
        """Returns (total loss, {task: loss value or None})."""
        device = batch["img"].device
        parts = {t: None for t in TASKS}
        if "det" in self.tasks:
            parts["det"] = self._det(out["det"], batch, batch["has_det"].to(device))
        if "seg" in self.tasks:
            keep = batch["has_seg"].to(device)
            if keep.any():
                parts["seg"] = seg_loss(out["seg"][keep], batch["seg_mask"][keep])
        if "depth" in self.tasks:
            keep = batch["has_depth"].to(device)
            if keep.any():
                parts["depth"] = silog_loss(out["depth"][keep, 0], batch["depth"][keep])

        total = torch.zeros((), device=device)
        for i, t in enumerate(TASKS):
            if parts[t] is None:
                continue
            if self.weighting == "uncertainty":
                s = self.log_vars[i]
                total = total + 0.5 * torch.exp(-s) * parts[t] + 0.5 * s
            else:
                total = total + parts[t]
        return total, {t: (None if v is None else v.detach().item()) for t, v in parts.items()}
