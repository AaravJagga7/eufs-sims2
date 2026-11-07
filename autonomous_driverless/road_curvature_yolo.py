"""Baseline YOLO & Computer Vision Road Curvature Perception Model & Algorithm.

Provides:
1. Road corridor & boundary segmentation from camera imagery.
2. Inverse Perspective Mapping (IPM) & 3D back-projection into vehicle ground frame.
3. Robust 2nd-order polynomial curve fitting: Y(X) = a*X^2 + b*X + c.
4. Analytical curvature estimation: kappa = |2a| / (1 + (2aX + b)^2)^(3/2).
5. Curvature radius R = 1 / kappa and direction classification (LEFT / RIGHT / STRAIGHT).
6. Formula Student curvature-based speed recommendation: v_safe = sqrt(a_lat_max / kappa).
7. Visual projection overlay drawing the predicted curvature arc onto the camera view.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math
import numpy as np
import cv2


@dataclass
class CurvatureResult:
    """Road curvature estimation output."""
    curvature: float              # Curvature kappa in 1/m
    radius_m: float               # Radius of curvature R in meters (capped at 500m for straight)
    direction: str                # "STRAIGHT", "LEFT", "RIGHT"
    curve_severity: str           # "STRAIGHT", "GENTLE", "MEDIUM", "SHARP"
    lateral_offset_m: float       # Vehicle lateral offset from track centerline at nose (m)
    heading_error_rad: float      # Tangent heading error relative to road centerline (rad)
    recommended_speed_mps: float  # Safe speed based on curvature dynamics (m/s)
    recommended_speed_kmh: float  # Safe speed (km/h)
    poly_coeffs: Tuple[float, float, float]  # (a, b, c) where Y = a*X^2 + b*X + c
    bev_centerline: np.ndarray    # Array of (X_veh, Y_veh) points in meters
    confidence: float             # Estimation confidence score [0.0, 1.0]


class RoadCurvatureYOLO:
    """YOLO & Computer Vision Road Curvature Perception & Analysis Engine."""

    def __init__(self, weights_path: Optional[str] = None,
                 a_lat_max: float = 6.5,
                 v_max_mps: float = 8.5):
        self.weights_path = weights_path
        self.a_lat_max = a_lat_max  # Maximum lateral acceleration (m/s^2) for ADS-DV racecar
        self.v_max_mps = v_max_mps  # Top speed cap (m/s, ~30 km/h)
        self.model = None

        if weights_path is not None:
            try:
                from ultralytics import YOLO
                self.model = YOLO(weights_path)
                print(f"[RoadCurvatureYOLO] Loaded YOLO perception weights: {weights_path}")
            except Exception as e:
                print(f"[RoadCurvatureYOLO] Fallback to native perception pipeline ({e})")

        # Temporal smoothing filter for curvature estimates
        self.filtered_a = 0.0
        self.filtered_b = 0.0
        self.filtered_c = 0.0
        self.has_history = False

    def estimate_curvature(self, img_bgr: np.ndarray,
                           depth_map: Optional[np.ndarray] = None,
                           fx: float = 502.5,
                           fy: float = 502.5,
                           cx: float = 320.0,
                           cy: float = 192.0,
                           cam_height: float = 0.76,
                           pitch_rad: float = 0.087) -> CurvatureResult:
        """Estimates road curvature from front camera view.
        
        Args:
            img_bgr: (H, W, 3) BGR image
            depth_map: Optional (H, W) metric depth map in meters
            fx, fy, cx, cy: Camera intrinsics
            cam_height: Camera mount height above ground (m)
            pitch_rad: Camera downward pitch angle (rad)
        """
        H, W = img_bgr.shape[:2]

        # 1. Segment Road Corridor / Lane Boundaries
        bev_pts = self._extract_track_centerline_points(img_bgr, depth_map,
                                                         fx, fy, cx, cy,
                                                         cam_height, pitch_rad)

        if len(bev_pts) < 5:
            # Default straight ahead if no clear points found
            return CurvatureResult(
                curvature=0.0,
                radius_m=500.0,
                direction="STRAIGHT",
                curve_severity="STRAIGHT",
                lateral_offset_m=0.0,
                heading_error_rad=0.0,
                recommended_speed_mps=self.v_max_mps,
                recommended_speed_kmh=self.v_max_mps * 3.6,
                poly_coeffs=(0.0, 0.0, 0.0),
                bev_centerline=np.zeros((0, 2), dtype=np.float32),
                confidence=0.20
            )

        # 2. Fit 2nd-degree polynomial: Y_veh = a * X_veh^2 + b * X_veh + c
        # In vehicle body frame: X is forward, Y is lateral (positive left)
        X = bev_pts[:, 0]
        Y = bev_pts[:, 1]

        # Robust polynomial fit with Huber/RANSAC or weighted least squares
        # Weight closer points slightly higher for lateral offset accuracy
        weights = 1.0 / np.clip(X, 1.0, 30.0)
        try:
            poly = np.polyfit(X, Y, deg=2, w=weights)
            a, b, c = float(poly[0]), float(poly[1]), float(poly[2])
        except Exception:
            poly = np.polyfit(X, Y, deg=1, w=weights)
            a, b, c = 0.0, float(poly[0]), float(poly[1])

        # 3. Temporal exponential smoothing
        if self.has_history:
            alpha = 0.35  # Smoothing factor
            a = alpha * a + (1.0 - alpha) * self.filtered_a
            b = alpha * b + (1.0 - alpha) * self.filtered_b
            c = alpha * c + (1.0 - alpha) * self.filtered_c
        else:
            self.has_history = True

        self.filtered_a = a
        self.filtered_b = b
        self.filtered_c = c

        # 4. Analytical Road Curvature Calculation
        # At X = 0 (vehicle nose): kappa = |2a| / (1 + b^2)^(3/2)
        denom = (1.0 + b**2)**1.5
        kappa = abs(2.0 * a) / max(1e-6, denom)
        
        # Radius of curvature: R = 1 / kappa
        if kappa < 1e-4:
            radius = 500.0
            direction = "STRAIGHT"
            severity = "STRAIGHT"
        else:
            radius = min(500.0, 1.0 / kappa)
            # a > 0 implies curve bends to the left; a < 0 bends to the right
            if abs(a) < 0.003:
                direction = "STRAIGHT"
                severity = "STRAIGHT"
            elif a > 0:
                direction = "LEFT"
                severity = "SHARP" if radius < 25.0 else ("MEDIUM" if radius < 50.0 else "GENTLE")
            else:
                direction = "RIGHT"
                severity = "SHARP" if radius < 25.0 else ("MEDIUM" if radius < 50.0 else "GENTLE")

        # Lateral offset & heading tangent error
        lat_offset = c
        heading_err = math.atan(b)

        # 5. Formula Student Speed Profiling
        # v_safe = sqrt(a_lat_max / kappa)
        if kappa > 1e-4:
            v_safe = min(self.v_max_mps, math.sqrt(self.a_lat_max / kappa))
            v_safe = max(3.5, v_safe)  # Keep minimum momentum (~12 km/h)
        else:
            v_safe = self.v_max_mps

        # Generate smooth sampled centerline in BEV
        xs = np.linspace(1.0, max(12.0, np.max(X)), 25)
        ys = a * xs**2 + b * xs + c
        centerline_bev = np.column_stack((xs, ys))

        conf = float(np.clip(len(bev_pts) / 40.0, 0.4, 0.98))

        return CurvatureResult(
            curvature=float(kappa),
            radius_m=float(radius),
            direction=direction,
            curve_severity=severity,
            lateral_offset_m=float(lat_offset),
            heading_error_rad=float(heading_err),
            recommended_speed_mps=float(v_safe),
            recommended_speed_kmh=float(v_safe * 3.6),
            poly_coeffs=(float(a), float(b), float(c)),
            bev_centerline=centerline_bev,
            confidence=conf
        )

    def _extract_track_centerline_points(self, img_bgr: np.ndarray,
                                        depth_map: Optional[np.ndarray],
                                        fx: float, fy: float, cx: float, cy: float,
                                        cam_height: float, pitch_rad: float) -> np.ndarray:
        """Extracts 3D ground points along the perceived track corridor."""
        H, W = img_bgr.shape[:2]
        pts_3d = []

        # Convert to HSV and grayscale
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        # Region of interest: road below the horizon
        horizon_v = int(cy + fy * math.tan(-pitch_rad))
        horizon_v = max(30, min(H - 60, horizon_v))

        # 1. Detect yellow track boundaries / cones (right boundary)
        # HSV Yellow: H: [18, 36], S: [90, 255], V: [90, 255]
        yellow_mask = cv2.inRange(hsv[horizon_v:, :], np.array([16, 80, 80]), np.array([38, 255, 255]))
        
        # 2. Detect blue track cones (left boundary)
        # HSV Blue: H: [95, 130], S: [80, 255], V: [60, 255]
        blue_mask = cv2.inRange(hsv[horizon_v:, :], np.array([95, 80, 60]), np.array([135, 255, 255]))

        # 3. Detect white dashed centerline markings
        # High value & low saturation in asphalt road area
        white_mask = cv2.inRange(hsv[horizon_v:, :], np.array([0, 0, 180]), np.array([180, 50, 255]))

        # Scan row slices from near-field to far-field
        step_v = max(4, (H - horizon_v) // 18)
        for v_sub in range(H - horizon_v - 6, 0, -step_v):
            v_img = horizon_v + v_sub
            
            # Find candidate centerline points at this row
            # Method A: Direct white lane marking detection
            row_white = np.where(white_mask[v_sub, :])[0]
            # Method B: Midpoint between left blue cones and right yellow cones
            row_blue = np.where(blue_mask[v_sub, :])[0]
            row_yellow = np.where(yellow_mask[v_sub, :])[0]

            u_center = None
            if len(row_blue) > 0 and len(row_yellow) > 0:
                u_center = 0.5 * (np.mean(row_blue) + np.mean(row_yellow))
            elif len(row_white) > 0:
                u_center = float(np.median(row_white))
            elif len(row_yellow) > 0:
                # Track width is ~3.2m -> estimate center offset to the left of yellow
                u_center = float(np.mean(row_yellow)) - (W * 0.18)
            elif len(row_blue) > 0:
                # Estimate center offset to the right of blue
                u_center = float(np.mean(row_blue)) + (W * 0.18)

            if u_center is not None and 10 <= u_center < W - 10:
                # 3D Back-projection: Image (u, v) -> Vehicle Frame (X, Y)
                if depth_map is not None:
                    # Use metric stereo depth if valid
                    d_val = depth_map[v_img, int(u_center)]
                    if 1.0 < d_val < 35.0:
                        Z_c = float(d_val)
                        X_c = (u_center - cx) * Z_c / fx
                        Y_c = (v_img - cy) * Z_c / fy
                        
                        cos_p = math.cos(pitch_rad)
                        sin_p = math.sin(pitch_rad)
                        x_veh = Z_c * cos_p + Y_c * sin_p
                        y_veh = -X_c
                        pts_3d.append((x_veh, y_veh))
                        continue

                # Analytical Flat-Ground IPM Back-Projection
                # Angle of pixel ray below optical axis
                phi = math.atan2(v_img - cy, fy)
                # Angle relative to ground plane
                gamma = pitch_rad + phi
                if gamma > 0.02:
                    d_ground = cam_height / math.sin(gamma)
                    X_forward = d_ground * math.cos(gamma)
                    Y_lateral = -(u_center - cx) * d_ground / fx
                    if 1.0 <= X_forward <= 35.0:
                        pts_3d.append((X_forward, Y_lateral))

        if len(pts_3d) < 4:
            # Fallback scan across asphalt road intensity boundaries
            road_roi = gray[horizon_v:, :]
            for v_sub in range(H - horizon_v - 8, 0, -step_v * 2):
                v_img = horizon_v + v_sub
                row = road_roi[v_sub, :]
                asphalt_pixels = np.where((row > 50) & (row < 130))[0]
                if len(asphalt_pixels) > 20:
                    u_center = float(np.median(asphalt_pixels))
                    phi = math.atan2(v_img - cy, fy)
                    gamma = pitch_rad + phi
                    if gamma > 0.02:
                        d_ground = cam_height / math.sin(gamma)
                        X_forward = d_ground * math.cos(gamma)
                        Y_lateral = -(u_center - cx) * d_ground / fx
                        if 1.0 <= X_forward <= 30.0:
                            pts_3d.append((X_forward, Y_lateral))

        return np.array(pts_3d, dtype=np.float32)

    def draw_curvature_overlay(self, img_bgr: np.ndarray,
                              result: CurvatureResult,
                              fx: float = 502.5, fy: float = 502.5,
                              cx: float = 320.0, cy: float = 192.0,
                              cam_height: float = 0.76,
                              pitch_rad: float = 0.087) -> np.ndarray:
        """Draws the estimated road curvature arc and telemetry badge on camera image."""
        vis = img_bgr.copy()
        H, W = vis.shape[:2]

        # 1. Project the polynomial centerline curve onto camera image
        if len(result.bev_centerline) > 2:
            img_pts = []
            cos_p = math.cos(pitch_rad)
            sin_p = math.sin(pitch_rad)
            
            for pt in result.bev_centerline:
                X_veh, Y_veh = pt[0], pt[1]
                # Convert vehicle frame back to camera optical frame
                Z_c = X_veh * cos_p + cam_height * sin_p
                X_c = -Y_veh
                Y_c = cam_height * cos_p - X_veh * sin_p
                
                if Z_c > 0.5:
                    u = int(cx + fx * X_c / Z_c)
                    v = int(cy + fy * Y_c / Z_c)
                    if 0 <= u < W and 0 <= v < H:
                        img_pts.append((u, v))

            # Draw smooth trajectory ribbon
            if len(img_pts) > 2:
                # Color based on curvature direction
                if result.direction == "LEFT":
                    line_color = (255, 180, 30)   # Cyan / Light Blue
                elif result.direction == "RIGHT":
                    line_color = (50, 200, 255)   # Orange / Yellow
                else:
                    line_color = (80, 255, 120)   # Green for straight

                pts_arr = np.array(img_pts, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(vis, [pts_arr], isClosed=False, color=line_color, thickness=4, lineType=cv2.LINE_AA)

                # Draw corridor width guides (track boundaries)
                for pt in img_pts[::3]:
                    cv2.circle(vis, pt, 3, (0, 255, 255), -1)

        # 2. Curvature Telemetry Badge (Top-Center)
        badge_w, badge_h = 240, 52
        bx1 = (W - badge_w) // 2
        by1 = 8
        bx2 = bx1 + badge_w
        by2 = by1 + badge_h

        overlay = vis.copy()
        cv2.rectangle(overlay, (bx1, by1), (bx2, by2), (18, 18, 24), -1)
        cv2.addWeighted(overlay, 0.78, vis, 0.22, 0, vis)
        cv2.rectangle(vis, (bx1, by1), (bx2, by2), (0, 220, 255), 1)

        # Direction arrow / text
        if result.direction == "LEFT":
            dir_symbol = "<- LEFT"
            dir_color = (255, 180, 30)
        elif result.direction == "RIGHT":
            dir_symbol = "RIGHT ->"
            dir_color = (50, 200, 255)
        else:
            dir_symbol = "|| STRAIGHT"
            dir_color = (80, 255, 120)

        cv2.putText(vis, f"CURVE: {dir_symbol} ({result.curve_severity})",
                    (bx1 + 10, by1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, dir_color, 1, cv2.LINE_AA)

        rad_str = f"R={result.radius_m:.1f}m" if result.radius_m < 300 else "R > 300m"
        cv2.putText(vis, f"{rad_str} | kappa={result.curvature:.3f} | v_rec={result.recommended_speed_kmh:.0f}km/h",
                    (bx1 + 10, by1 + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1, cv2.LINE_AA)

        return vis

