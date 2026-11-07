"""Combined model: one shared YOLOv11 body with three heads.

  image -> YOLOv11 backbone + neck -> P3 (1/8), P4 (1/16), P5 (1/32) features
                                        |-> detection head  (YOLO's own, 8 classes)
                                        |-> road head       (2 classes, full size)
                                        |-> distance head   (metres, full size)

Input: float32 [B, 3, 384, 640], RGB, values 0-1 (exactly what the data loader gives).
forward() returns {"det": ..., "seg": logits [B, 2, H, W] or None, "depth": [B, 1, H, W] or None}
  det in train mode: YOLO's raw prediction dict (what the YOLO loss needs)
  det in eval mode:  (decoded boxes [B, 4 + nc, anchors], raw dict)
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
from ultralytics.cfg import get_cfg
from ultralytics.nn.tasks import DetectionModel

from datasets.contract import DEPTH_MAX_M, NUM_DET_CLASSES, NUM_SEG_CLASSES

CLASS_NAMES = ("pothole", "hump", "car", "two_wheeler", "bus_truck", "person",
               "traffic_light", "traffic_sign")   # same order as configs/classes.yaml


def conv_bn_relu(c_in, c_out, k=3):
    return nn.Sequential(nn.Conv2d(c_in, c_out, k, padding=k // 2, bias=False),
                         nn.BatchNorm2d(c_out), nn.ReLU(inplace=True))


class DenseHead(nn.Module):
    """Light FPN decoder: fuses P3-P5 into one map at 1/4 size, then predicts per pixel."""

    def __init__(self, in_channels, out_channels, mid=128):
        super().__init__()
        self.lateral = nn.ModuleList(nn.Conv2d(c, mid, 1) for c in in_channels)
        self.fuse = nn.Sequential(conv_bn_relu(mid, mid), conv_bn_relu(mid, mid))
        self.refine = conv_bn_relu(mid, mid // 2)
        self.out = nn.Conv2d(mid // 2, out_channels, 1)

    def forward(self, feats, out_size):
        size = feats[0].shape[-2:]                       # P3 size (1/8)
        x = sum(F.interpolate(lat(f), size=size, mode="bilinear", align_corners=False)
                if f.shape[-2:] != size else lat(f)
                for lat, f in zip(self.lateral, feats))
        x = self.fuse(x)
        x = F.interpolate(x, scale_factor=2, mode="bilinear", align_corners=False)  # 1/4
        x = self.out(self.refine(x))
        return F.interpolate(x, size=out_size, mode="bilinear", align_corners=False)


class MultiTaskNet(nn.Module):
    def __init__(self, yolo_cfg="yolo11s.yaml", weights="yolo11s.pt", pretrained=True,
                 tasks=("det", "seg", "depth")):
        super().__init__()
        self.tasks = tuple(tasks)
        self.yolo = DetectionModel(yolo_cfg, nc=NUM_DET_CLASSES, verbose=False)
        self.yolo.names = dict(enumerate(CLASS_NAMES))   # lets "car"/"person" reuse COCO rows
        if pretrained:
            from ultralytics import YOLO
            self.yolo.load(YOLO(weights).model)        # copies every weight whose shape matches
        self.yolo.args = get_cfg()                     # loss gains (box, cls, dfl) for YOLO's loss

        detect = self.yolo.model[-1]
        in_ch = [seq[0].conv.in_channels for seq in detect.cv2]   # P3, P4, P5 channels
        self._feats = None
        detect.register_forward_pre_hook(self._grab_feats)

        self.seg_head = DenseHead(in_ch, NUM_SEG_CLASSES) if "seg" in self.tasks else None
        self.depth_head = DenseHead(in_ch, 1) if "depth" in self.tasks else None

    def _grab_feats(self, module, inputs):
        self._feats = list(inputs[0])                  # the neck outputs going into Detect

    def forward(self, img):
        det = self.yolo(img)                           # also fills self._feats via the hook
        feats, size = self._feats, img.shape[-2:]
        seg = self.seg_head(feats, size) if self.seg_head is not None else None
        depth = None
        if self.depth_head is not None:
            depth = torch.sigmoid(self.depth_head(feats, size).float()) * DEPTH_MAX_M
        return {"det": det, "seg": seg, "depth": depth}

    def param_groups(self, lr_body, lr_heads, weight_decay):
        """Pretrained body at a lower learning rate than the detection, road and distance heads."""
        detect = self.yolo.model[-1]
        head_ids = {id(p) for p in detect.parameters()}
        body = [p for p in self.yolo.parameters() if id(p) not in head_ids]
        heads = list(detect.parameters())
        for h in (self.seg_head, self.depth_head):
            if h is not None:
                heads += list(h.parameters())
        return [{"params": body, "lr": lr_body, "weight_decay": weight_decay},
                {"params": heads, "lr": lr_heads, "weight_decay": weight_decay}]
