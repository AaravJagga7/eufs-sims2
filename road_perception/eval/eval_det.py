"""Detection evaluation (owned by Person B).

Computes mAP50 metrics:
- mAP50:         mean Average Precision across classes at IoU=0.50
- mAP50_pothole: AP50 for class 0 (pothole)
- mAP50_hump:    AP50 for class 1 (speed breaker / hump)

Model predict(images)["boxes"] contract (Rule 5):
List of detections per image: [x1, y1, x2, y2, score, cls_id] in pixel coordinates.

Run standalone:
  python -m eval.eval_det --dataset data_processed/speedbump --split test
"""
import argparse
import numpy as np
import torch
from torch.utils.data import DataLoader

from datasets.contract import IMG_H, IMG_W, collate_fn
from datasets.multitask_dataset import MultiTaskDataset


def box_iou_xyxy(box1, box2):
    """Compute IoU between two sets of boxes [N, 4] and [M, 4] in (x1, y1, x2, y2)."""
    if len(box1) == 0 or len(box2) == 0:
        return np.zeros((len(box1), len(box2)), dtype=np.float32)

    x1 = np.maximum(box1[:, None, 0], box2[None, :, 0])
    y1 = np.maximum(box1[:, None, 1], box2[None, :, 1])
    x2 = np.minimum(box1[:, None, 2], box2[None, :, 2])
    y2 = np.minimum(box1[:, None, 3], box2[None, :, 3])

    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area1 = (box1[:, 2] - box1[:, 0]) * (box1[:, 3] - box1[:, 1])
    area2 = (box2[:, 2] - box2[:, 0]) * (box2[:, 3] - box2[:, 1])
    union = area1[:, None] + area2[None, :] - intersection

    return intersection / np.maximum(union, 1e-6)


def compute_class_ap(detections, ground_truths, iou_thresh=0.50):
    """Computes AP at iou_thresh for a single class.
    detections: list of (x1, y1, x2, y2, score, img_id)
    ground_truths: dict mapping img_id -> list of (x1, y1, x2, y2)
    """
    total_gts = sum(len(gts) for gts in ground_truths.values())
    if total_gts == 0:
        return float("nan")
    if len(detections) == 0:
        return 0.0

    # Sort detections by score descending
    detections = sorted(detections, key=lambda d: d[4], reverse=True)

    matched_gt = {img_id: np.zeros(len(gts), dtype=bool) for img_id, gts in ground_truths.items()}

    tp = np.zeros(len(detections))
    fp = np.zeros(len(detections))

    for i, det in enumerate(detections):
        img_id = det[5]
        gts = ground_truths.get(img_id, [])
        if len(gts) == 0:
            fp[i] = 1.0
            continue

        gt_boxes = np.array(gts)
        det_box = np.array(det[:4]).reshape(1, 4)
        ious = box_iou_xyxy(det_box, gt_boxes)[0]

        best_idx = np.argmax(ious)
        best_iou = ious[best_idx]

        if best_iou >= iou_thresh:
            if not matched_gt[img_id][best_idx]:
                tp[i] = 1.0
                matched_gt[img_id][best_idx] = True
            else:
                fp[i] = 1.0
        else:
            fp[i] = 1.0

    cum_tp = np.cumsum(tp)
    cum_fp = np.cumsum(fp)
    recalls = cum_tp / total_gts
    precisions = cum_tp / (cum_tp + cum_fp)

    # 11-point interpolated AP or all-points PR AUC
    mrec = np.concatenate(([0.0], recalls, [1.0]))
    mpre = np.concatenate(([0.0], precisions, [0.0]))
    for k in range(len(mpre) - 2, -1, -1):
        mpre[k] = max(mpre[k], mpre[k + 1])

    # Integrate area under curve
    indices = np.where(mrec[1:] != mrec[:-1])[0]
    ap = float(np.sum((mrec[indices + 1] - mrec[indices]) * mpre[indices + 1]))
    return ap


def evaluate_detection(model, data_root="data_processed/speedbump", split="test", batch_size=4):
    """Evaluates any model with predict(images) on detection."""
    ds = MultiTaskDataset(data_root, split)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=False, collate_fn=collate_fn)

    # Dictionary per class: list of detections (x1, y1, x2, y2, score, img_id)
    class_detections = {}
    class_gts = {}

    img_counter = 0

    for batch in loader:
        imgs = batch["img"]
        b = imgs.shape[0]
        res = model.predict(imgs)
        pred_boxes_batch = res.get("boxes")
        if pred_boxes_batch is None:
            return {}

        # Process each sample in batch
        for i in range(b):
            img_id = img_counter
            img_counter += 1

            # Extract ground truths for this sample
            # Batch det_targets: (batch_idx, cls, cx, cy, w, h)
            sample_gts = batch["det_targets"][batch["det_targets"][:, 0] == i]
            for row in sample_gts:
                cls_id = int(row[1].item())
                cx, cy, w, h = row[2:].numpy()
                x1 = (cx - w / 2.0) * IMG_W
                y1 = (cy - h / 2.0) * IMG_H
                x2 = (cx + w / 2.0) * IMG_W
                y2 = (cy + h / 2.0) * IMG_H
                class_gts.setdefault(cls_id, {}).setdefault(img_id, []).append([x1, y1, x2, y2])

            # Extract predictions
            if pred_boxes_batch is not None and len(pred_boxes_batch) > i:
                sample_preds = pred_boxes_batch[i]
                if sample_preds is not None:
                    if isinstance(sample_preds, torch.Tensor):
                        sample_preds = sample_preds.detach().cpu().numpy()
                    for det in sample_preds:
                        # [x1, y1, x2, y2, score, cls_id]
                        x1, y1, x2, y2, score, cls_id = det
                        class_detections.setdefault(int(cls_id), []).append(
                            [float(x1), float(y1), float(x2), float(y2), float(score), img_id]
                        )

    # Compute AP per class
    all_classes = sorted(set(list(class_gts.keys()) + list(class_detections.keys())))
    aps = {}
    for c in all_classes:
        gts = class_gts.get(c, {})
        dets = class_detections.get(c, [])
        ap = compute_class_ap(dets, gts, iou_thresh=0.50)
        aps[c] = ap

    valid_aps = [v for v in aps.values() if not np.isnan(v)]
    map50 = float(np.mean(valid_aps)) if valid_aps else 0.0

    metrics = {
        "mAP50": map50,
        "mAP50_pothole": aps.get(0, 0.0) if not np.isnan(aps.get(0, float("nan"))) else 0.0,
        "mAP50_hump": aps.get(1, 0.0) if not np.isnan(aps.get(1, float("nan"))) else 0.0,
    }
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data_processed/speedbump")
    parser.add_argument("--split", default="test")
    args = parser.parse_args()

    # Dummy perfect detector for verification
    class PerfectDetector:
        def predict(self, imgs):
            return {"boxes": [], "seg": None, "depth": None}

    model = PerfectDetector()
    metrics = evaluate_detection(model, data_root=args.dataset, split=args.split)
    print("Detection evaluation results:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")
