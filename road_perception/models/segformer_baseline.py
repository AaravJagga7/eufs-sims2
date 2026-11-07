"""SegFormer road baseline: road vs not-road (2 classes).

Takes images exactly as the data loader gives them (float32 [B, 3, 384, 640],
values 0-1). ImageNet normalisation happens inside the model, and the output
is upsampled to full image size, so callers never deal with either.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers import SegformerConfig, SegformerForSemanticSegmentation

from datasets.contract import NUM_SEG_CLASSES

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


class SegFormerRoad(nn.Module):
    def __init__(self, backbone="nvidia/mit-b0", pretrained=True):
        super().__init__()
        if pretrained:
            # Downloads ImageNet-pretrained encoder weights; the road head starts untrained
            self.net = SegformerForSemanticSegmentation.from_pretrained(
                backbone, num_labels=NUM_SEG_CLASSES)
        else:
            # Same architecture with no download (used when loading our own checkpoint).
            # SegformerConfig's defaults are the MiT-B0 sizes.
            if backbone != "nvidia/mit-b0":
                raise ValueError("pretrained=False only supports nvidia/mit-b0")
            self.net = SegformerForSemanticSegmentation(
                SegformerConfig(num_labels=NUM_SEG_CLASSES))
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))

    def forward(self, img):
        x = (img - self.mean) / self.std
        logits = self.net(pixel_values=x).logits            # [B, 2, H/4, W/4]
        return F.interpolate(logits, size=img.shape[-2:], mode="bilinear",
                             align_corners=False)          # [B, 2, H, W]

    def param_groups(self, lr, head_lr_mult=10.0):
        """Encoder at lr, freshly initialised head at lr x head_lr_mult."""
        return [
            {"params": self.net.segformer.parameters(), "lr": lr},
            {"params": self.net.decode_head.parameters(), "lr": lr * head_lr_mult},
        ]
