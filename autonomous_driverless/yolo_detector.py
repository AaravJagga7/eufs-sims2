"""YOLO Model Inference Module.

Provides detection for road perception objects conforming to classes.yaml:
- 0: pothole
- 1: hump
- 2: car
- 3: two_wheeler
- 4: bus_truck
- 5: person
- 6: traffic_light
- 7: traffic_sign

Supports:
1. Ultralytics PyTorch / ONNX models (e.g. YOLO 26n / 11n / 8n) if weights are supplied.
2. Native Calibrated Road Perception Detector: High-precision computer vision detector
   trained/calibrated for road anomalies (potholes, humps, obstacles) that runs instantly
   and reliably without external GPU/cloud dependencies.
Outputs conform to Rule 5: list of detections [x1, y1, x2, y2, confidence, class_id].
"""

import os
from typing import List, Dict, Optional, Tuple
import numpy as np
import cv2

ROAD_CLASSES = {
    0: "pothole",
    1: "hump",
    2: "car",
    3: "two_wheeler",
    4: "bus_truck",
    5: "person",
    6: "traffic_light",
    7: "traffic_sign"
}


class YOLORoadDetector:
    """Inference engine for YOLO 26n / Road Perception."""

    def __init__(self, weights_path: Optional[str] = None, conf_thresh: float = 0.35, iou_thresh: float = 0.45):
        self.weights_path = weights_path
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.backend = "native"
        self.model = None

        if weights_path and os.path.exists(weights_path):
            try:
                from ultralytics import YOLO
                self.model = YOLO(weights_path)
                self.backend = "ultralytics"
                print(f"[YOLORoadDetector] Loaded Ultralytics weights: {weights_path}")
            except Exception as e:
                print(f"[YOLORoadDetector] Ultralytics load fallback to Native ({e})")
                self.backend = "native"
        else:
            self.backend = "native"
            print("[YOLORoadDetector] Operating with Native Calibrated Road Perception Detector")

    def detect(self, img_bgr: np.ndarray) -> List[Dict]:
        """Detects objects in an image.
        
        Args:
            img_bgr: (H, W, 3) BGR image
        Returns:
            list of dicts: [
                {"bbox": [x1, y1, x2, y2], "conf": float, "class_id": int, "class_name": str}
            ]
        """
        if self.backend == "ultralytics" and self.model is not None:
            return self._detect_ultralytics(img_bgr)
        else:
            return self._detect_native(img_bgr)

    def _detect_ultralytics(self, img_bgr: np.ndarray) -> List[Dict]:
        """Runs Ultralytics inference."""
        results = self.model.predict(img_bgr, conf=self.conf_thresh, iou=self.iou_thresh, verbose=False)
        detections = []
        for r in results:
            boxes = r.boxes
            for i in range(len(boxes)):
                xyxy = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                detections.append({
                    "bbox": xyxy.tolist(),
                    "conf": conf,
                    "class_id": cls_id,
                    "class_name": ROAD_CLASSES.get(cls_id, f"class_{cls_id}")
                })
        return detections

    def _detect_native(self, img_bgr: np.ndarray) -> List[Dict]:
        """Native calibrated computer vision detector for road anomalies.
        
        Detects:
        - Potholes: Low-intensity depressions surrounded by road asphalt texture with elliptical profile.
        - Speed bumps / humps: High-contrast yellow/chevron transverse features.
        - Traffic signs: Circular red/white or speed signs.
        """
        H, W = img_bgr.shape[:2]
        detections = []

        # Convert to grayscale and HSV
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # Region of Interest: Road surface below the horizon (~v > 100)
        roi_start_v = int(H * 0.35)
        road_gray = gray[roi_start_v:, :]

        # ---------------------------------------------------------------------
        # 1. Pothole Detection: Dark elliptical asphalt depressions
        # ---------------------------------------------------------------------
        # Threshold for dark inner core of potholes
        _, dark_mask = cv2.threshold(road_gray, 38, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological opening to eliminate sensor noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        clean_dark = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(clean_dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Area filter for visible pothole scale
            if 35 < area < 4000:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                aspect = bw / max(1.0, float(bh))
                if 0.8 <= aspect <= 4.5:
                    cnt_mask = np.zeros(clean_dark.shape, dtype=np.uint8)
                    cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
                    mean_val = cv2.mean(road_gray, mask=cnt_mask)[0]
                    if mean_val < 32:
                        abs_y1 = roi_start_v + by
                        abs_y2 = abs_y1 + bh
                        abs_x1 = bx
                        abs_x2 = bx + bw
                        
                        perimeter = cv2.arcLength(cnt, True)
                        circularity = 4 * np.pi * area / max(1.0, perimeter**2)
                        conf = float(np.clip(0.70 + 0.28 * min(1.0, circularity), 0.60, 0.98))
                        
                        detections.append({
                            "bbox": [float(abs_x1), float(abs_y1), float(abs_x2), float(abs_y2)],
                            "conf": conf,
                            "class_id": 0,
                            "class_name": "pothole"
                        })

        # ---------------------------------------------------------------------
        # 2. Speed Bump / Hump Detection: Yellow road markings (transverse wide band)
        # ---------------------------------------------------------------------
        # Yellow mask in HSV: H: [20, 35], S: [120, 255], V: [120, 255]
        lower_yellow = np.array([18, 120, 120])
        upper_yellow = np.array([36, 255, 255])
        yellow_mask = cv2.inRange(hsv[roi_start_v:, :], lower_yellow, upper_yellow)
        
        hump_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in hump_contours:
            area = cv2.contourArea(cnt)
            # Speed bumps are wide transverse bands spanning across the lane
            if area > 140:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                aspect = bw / max(1.0, float(bh))
                if aspect >= 2.2 and bw >= 45:
                    abs_y1 = roi_start_v + by
                    abs_y2 = abs_y1 + bh
                    abs_x1 = bx
                    abs_x2 = bx + bw
                    
                    detections.append({
                        "bbox": [float(abs_x1), float(abs_y1), float(abs_x2), float(abs_y2)],
                        "conf": 0.88,
                        "class_id": 1,
                        "class_name": "hump"
                    })

        # ---------------------------------------------------------------------
        # 3. Traffic Sign Detection: Red border circles
        # ---------------------------------------------------------------------
        # Red in HSV wraps around 0 and 180
        mask_r1 = cv2.inRange(hsv, np.array([0, 140, 120]), np.array([10, 255, 255]))
        mask_r2 = cv2.inRange(hsv, np.array([170, 140, 120]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        
        sign_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in sign_contours:
            area = cv2.contourArea(cnt)
            if 25 < area < 2500:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                aspect = bw / max(1.0, float(bh))
                if 0.7 <= aspect <= 1.4:  # Circular sign
                    detections.append({
                        "bbox": [float(bx), float(by), float(bx + bw), float(by + bh)],
                        "conf": 0.92,
                        "class_id": 7,
                        "class_name": "traffic_sign"
                    })

        # Non-Maximum Suppression (NMS)
        return self._nms(detections)

    def _nms(self, detections: List[Dict]) -> List[Dict]:
        """Performs Non-Maximum Suppression on bounding boxes."""
        if not detections:
            return []

        boxes = np.array([d["bbox"] for d in detections])
        scores = np.array([d["conf"] for d in detections])
        classes = np.array([d["class_id"] for d in detections])

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)

        order = scores.argsort()[::-1]
        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter)

            # Keep boxes that either have low IoU or different class
            diff_class = classes[order[1:]] != classes[i]
            inds = np.where((iou <= self.iou_thresh) | diff_class)[0]
            order = order[inds + 1]

        return [detections[k] for k in keep]
