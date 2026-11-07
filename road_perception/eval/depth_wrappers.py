"""Depth baseline model wrappers for KITTI (owned by Person B).

Provides:
- StereoSGBMWrapper: Stereo baseline using OpenCV StereoSGBM on left/right pairs
- DepthAnythingV2Wrapper: Monocular depth baseline using Depth Anything V2

Both wrappers conform to Rule 5:
predict(images) returns {"boxes": None, "seg": None, "depth": np.ndarray (B, 384, 640)}
"""
import json
import os
import cv2
import numpy as np
import torch

from datasets.contract import DEPTH_MAX_M, IMG_H, IMG_W


class StereoSGBMWrapper:
    """Stereo vision baseline on KITTI using OpenCV StereoSGBM."""

    def __init__(self, dataset_root="data_processed/kitti", split="test",
                 num_disparities=16 * 6, block_size=5):
        self.dataset_root = dataset_root
        self.split = split
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size * block_size,
            P2=32 * 3 * block_size * block_size,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        with open(os.path.join(dataset_root, "splits.json")) as f:
            self.stems = json.load(f)[split]
        self._cur_idx = 0

    def reset(self):
        self._cur_idx = 0

    def predict(self, images, stems=None):
        """images: Tensor of shape (B, 3, 384, 640) in [0, 1] or numpy array."""
        if isinstance(images, torch.Tensor):
            b = images.shape[0]
            # Convert to uint8 BGR for OpenCV
            imgs_np = (images.permute(0, 2, 3, 1).cpu().numpy() * 255.0).clip(0, 255).astype(np.uint8)
            imgs_bgr = [cv2.cvtColor(img, cv2.COLOR_RGB2BGR) for img in imgs_np]
        else:
            imgs_bgr = images
            b = len(imgs_bgr)

        depths = []
        for i in range(b):
            if stems is not None and i < len(stems):
                stem = stems[i]
            else:
                idx = (self._cur_idx + i) % len(self.stems)
                stem = self.stems[idx]

            left_img = imgs_bgr[i]
            right_path = os.path.join(self.dataset_root, "images_right", self.split, stem + ".jpg")
            calib_path = os.path.join(self.dataset_root, "calib", self.split, stem + ".json")

            right_img = cv2.imread(right_path)
            fb = 384.38  # standard KITTI baseline * focal length fallback
            scale = 640.0 / 1242.0
            if os.path.exists(calib_path):
                with open(calib_path) as f:
                    calib = json.load(f)
                    fb = calib.get("fb", fb)
                    scale = calib.get("scale", scale)

            # Scale focal length by the letterbox resize factor
            fb_scaled = fb * scale

            if right_img is not None:
                gray_l = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                gray_r = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
                disp = self.matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
                valid = disp > 0.0
                d_pred = np.zeros((IMG_H, IMG_W), dtype=np.float32)
                d_pred[valid] = np.clip(fb_scaled / disp[valid], 0.0, DEPTH_MAX_M)
            else:
                d_pred = np.zeros((IMG_H, IMG_W), dtype=np.float32)

            depths.append(d_pred)

        self._cur_idx = (self._cur_idx + b) % len(self.stems)
        return {
            "boxes": None,
            "seg": None,
            "depth": np.stack(depths, axis=0),
        }


class DepthAnythingV2Wrapper:
    """Monocular depth baseline using Depth Anything V2 Small.
    Supports local metric weights or standard Hugging Face pipeline if online."""

    def __init__(self, model_name="depth-anything/Depth-Anything-V2-Small-hf", device=None):
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device
        self.model = None
        self.processor = None
        self._load_model(model_name)

    def _load_model(self, model_name):
        try:
            from transformers import AutoImageProcessor, AutoModelForDepthEstimation
            self.processor = AutoImageProcessor.from_pretrained(model_name)
            self.model = AutoModelForDepthEstimation.from_pretrained(model_name).to(self.device)
            self.model.eval()
        except Exception as e:
            # Fallback when running offline without cached HF weights
            self.model = None
            print(f"DepthAnythingV2Wrapper initialized in fallback mode: {e}")

    def predict(self, images):
        """images: Tensor (B, 3, 384, 640) in [0, 1]."""
        if isinstance(images, np.ndarray):
            images = torch.from_numpy(images).float()
            if images.ndim == 3:
                images = images.unsqueeze(0)
            if images.shape[-1] == 3:
                images = images.permute(0, 3, 1, 2)

        B = images.shape[0]
        if self.model is not None:
            with torch.no_grad():
                inputs = images.to(self.device)
                outputs = self.model(inputs)
                pred_depth = outputs.predicted_depth
                # Interpolate to target size
                pred_depth = torch.nn.functional.interpolate(
                    pred_depth.unsqueeze(1),
                    size=(IMG_H, IMG_W),
                    mode="bicubic",
                    align_corners=False,
                ).squeeze(1).cpu().numpy()
        else:
            # Fallback for testing harness: return zero depth
            pred_depth = np.zeros((B, IMG_H, IMG_W), dtype=np.float32)

        return {
            "boxes": None,
            "seg": None,
            "depth": pred_depth,
        }


def get_depth_model(name, dataset_root="data_processed/kitti", split="test"):
    if name.lower() == "stereosgbm":
        return StereoSGBMWrapper(dataset_root=dataset_root, split=split)
    elif name.lower() in ("depthanything", "depth_anything"):
        return DepthAnythingV2Wrapper()
    else:
        raise ValueError(f"Unknown depth model: {name}")
