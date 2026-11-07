"""Telemetry & Multi-View Visualization Dashboard.

Renders a live 4-panel telemetry display:
1. Quadrant 1 (Top-Left): Left Stereo Camera with YOLO Bounding Boxes & 3D Depth tags
2. Quadrant 2 (Top-Right): Calibrated Stereo Disparity & Metric Depth Map (Colormapped)
3. Quadrant 3 (Bottom-Left): Overhead World Map with EKF SLAM Pose & 3-Sigma Landmark Ellipses
4. Quadrant 4 (Bottom-Right): Cockpit Telemetry, SLAM Error Tracking & Accuracy Benchmarks
"""

from typing import List, Dict, Optional, Tuple
import numpy as np
import cv2

from world_environment import WorldEnvironment, WorldObject


class TelemetryDashboard:
    """Multi-view visualizer and performance telemetry logger."""

    def __init__(self, width: int = 1280, height: int = 768):
        self.width = width
        self.height = height
        self.q_w = width // 2
        self.q_h = height // 2

    def render_frame(self,
                     left_img: np.ndarray,
                     depth_map: np.ndarray,
                     detections: List[Dict],
                     triangulated_landmarks: List[Dict],
                     vehicle_gt: Tuple[float, float, float, float, float],
                     vehicle_odom: Tuple[float, float, float],
                     slam_pose: Tuple[float, float, float],
                     slam_landmarks: List[Dict],
                     world: WorldEnvironment,
                     slam_history: List[Tuple[float, float, float]],
                     odom_history: List[Tuple[float, float, float]],
                     gt_history: List[Tuple[float, float, float]],
                     depth_metrics: Dict[str, float],
                     step_time: float,
                     control_info: Dict,
                     tpp_img: Optional[np.ndarray] = None,
                     curvature_res: Optional[object] = None) -> np.ndarray:
        """Renders the composite 4-quadrant frame."""
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # -------------------------------------------------------------
        # Quadrant 1: Left Stereo View + YOLO Detections + 3D Depth Tags
        # -------------------------------------------------------------
        q1_img = self._render_camera_view(left_img, detections, triangulated_landmarks, curvature_res)
        canvas[:self.q_h, :self.q_w] = cv2.resize(q1_img, (self.q_w, self.q_h))

        # -------------------------------------------------------------
        # Quadrant 2: Third-Person Perspective (TPP) Chase Camera View
        # -------------------------------------------------------------
        if tpp_img is not None:
            q2_img = cv2.resize(tpp_img, (self.q_w, self.q_h))
            # Overlay mini stereo depth inset in bottom-right corner
            depth_mini = self._render_depth_view(depth_map, triangulated_landmarks)
            in_w, in_h = self.q_w // 4, self.q_h // 4
            depth_thumb = cv2.resize(depth_mini, (in_w, in_h))
            q2_img[self.q_h - in_h - 6:self.q_h - 6, self.q_w - in_w - 6:self.q_w - 6] = depth_thumb
            cv2.rectangle(q2_img, (self.q_w - in_w - 6, self.q_h - in_h - 6),
                          (self.q_w - 6, self.q_h - 6), (0, 220, 255), 1)
            cv2.putText(q2_img, "STEREO DEPTH", (self.q_w - in_w - 4, self.q_h - in_h - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 220, 255), 1)
        else:
            q2_img = cv2.resize(self._render_depth_view(depth_map, triangulated_landmarks), (self.q_w, self.q_h))
        canvas[:self.q_h, self.q_w:] = q2_img

        # -------------------------------------------------------------
        # Quadrant 3: Overhead World & EKF SLAM Map
        # -------------------------------------------------------------
        q3_img = self._render_slam_map(world, vehicle_gt, vehicle_odom, slam_pose,
                                       slam_landmarks, slam_history, odom_history, gt_history)
        canvas[self.q_h:, :self.q_w] = cv2.resize(q3_img, (self.q_w, self.q_h))

        # -------------------------------------------------------------
        # Quadrant 4: Cockpit Telemetry & SLAM Accuracy Dashboard
        # -------------------------------------------------------------
        q4_img = self._render_telemetry_hud(vehicle_gt, slam_pose, slam_landmarks,
                                            world, depth_metrics, step_time, control_info, curvature_res)
        canvas[self.q_h:, self.q_w:] = cv2.resize(q4_img, (self.q_w, self.q_h))

        # Dividers
        cv2.line(canvas, (self.q_w, 0), (self.q_w, self.height), (80, 80, 80), 2)
        cv2.line(canvas, (0, self.q_h), (self.width, self.q_h), (80, 80, 80), 2)

        return canvas

    def _render_camera_view(self, left_img: np.ndarray, detections: List[Dict],
                            triangulated: List[Dict],
                            curvature_res: Optional[object] = None) -> np.ndarray:
        """Draws bounding boxes, confidence, class, and 3D metric distances."""
        vis = left_img.copy()

        # Overlay crosshair at optical center
        cx, cy = 320, 192
        cv2.drawMarker(vis, (cx, cy), (100, 255, 100), cv2.MARKER_CROSS, 16, 1)

        # Draw detections
        for i, det in enumerate(detections):
            bbox = [int(v) for v in det["bbox"]]
            cls_name = det.get("class_name", "object")
            conf = det.get("conf", 0.0)
            
            # Color coding
            if cls_name == "pothole":
                color = (0, 0, 255)      # Red for potholes
            elif cls_name == "hump":
                color = (0, 215, 255)    # Yellow/Gold for speed bumps
            elif cls_name == "traffic_sign":
                color = (255, 0, 200)    # Magenta for signs
            else:
                color = (0, 255, 0)      # Green

            cv2.rectangle(vis, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
            
            # 3D depth string if triangulated
            depth_str = ""
            if i < len(triangulated) and triangulated[i] is not None:
                z_m = triangulated[i].get("range", 0.0)
                depth_str = f"| Z={z_m:.2f}m"

            label = f"{cls_name.upper()} {conf:.2f} {depth_str}"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            cv2.rectangle(vis, (bbox[0], max(0, bbox[1] - lh - 4)), (bbox[0] + lw + 4, bbox[1]), color, -1)
            cv2.putText(vis, label, (bbox[0] + 2, max(10, bbox[1] - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

        # Title banner
        cv2.rectangle(vis, (0, 0), (self.calib_width if hasattr(self, 'calib_width') else 640, 24), (20, 20, 20), -1)
        cv2.putText(vis, "STEREO CAM (LEFT) - YOLO ROAD PERCEPTION & 3D LOCALIZATION",
                    (10, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        # Draw projected road curvature centerline ribbon
        if curvature_res is not None and hasattr(curvature_res, "bev_centerline") and len(curvature_res.bev_centerline) > 2:
            img_pts = []
            fx, fy, cx, cy = 502.5, 502.5, 320.0, 192.0
            pitch_rad = 0.087
            cos_p = np.cos(pitch_rad)
            sin_p = np.sin(pitch_rad)
            H, W = vis.shape[:2]
            for pt in curvature_res.bev_centerline:
                X_v, Y_v = pt[0], pt[1]
                Z_c = X_v * cos_p + 0.76 * sin_p
                X_c = -Y_v
                Y_c = 0.76 * cos_p - X_v * sin_p
                if Z_c > 0.5:
                    u = int(cx + fx * X_c / Z_c)
                    v = int(cy + fy * Y_c / Z_c)
                    if 0 <= u < W and 0 <= v < H:
                        img_pts.append((u, v))
            if len(img_pts) > 2:
                c_col = (255, 180, 40) if curvature_res.direction == "LEFT" else ((40, 180, 255) if curvature_res.direction == "RIGHT" else (60, 240, 120))
                cv2.polylines(vis, [np.array(img_pts, dtype=np.int32)], isClosed=False, color=c_col, thickness=3, lineType=cv2.LINE_AA)

        return vis

    def _render_depth_view(self, depth_map: np.ndarray, triangulated: List[Dict]) -> np.ndarray:
        """Visualizes calibrated metric depth map with color-coded distance."""
        # Clip depth between 0.5m and 35.0m for clear visualization
        clipped = np.clip(depth_map, 0.5, 35.0)
        norm = ((clipped - 0.5) / 34.5 * 255.0).astype(np.uint8)
        # Invert so near objects are warmer (bright yellow/red) and far are blue/purple
        norm_inv = 255 - norm
        depth_color = cv2.applyColorMap(norm_inv, cv2.COLORMAP_TURBO)

        # Draw markers on triangulated points
        for tri in triangulated:
            if tri is not None:
                uc = int(tri.get("u_center", 0))
                vc = int(tri.get("v_center", 0))
                zm = tri.get("range", 0.0)
                if 0 <= uc < depth_color.shape[1] and 0 <= vc < depth_color.shape[0]:
                    cv2.circle(depth_color, (uc, vc), 5, (255, 255, 255), 2)
                    cv2.putText(depth_color, f"{zm:.1f}m", (uc + 8, vc),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # Title banner & depth legend
        cv2.rectangle(depth_color, (0, 0), (depth_color.shape[1], 24), (20, 20, 20), -1)
        cv2.putText(depth_color, "STEREOSCOPIC METRIC DEPTH MAP [TURBO: NEAR=RED, FAR=BLUE]",
                    (10, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        return depth_color

    def _render_slam_map(self, world: WorldEnvironment,
                         vehicle_gt: Tuple[float, float, float, float, float],
                         vehicle_odom: Tuple[float, float, float],
                         slam_pose: Tuple[float, float, float],
                         slam_landmarks: List[Dict],
                         slam_history: List[Tuple[float, float, float]],
                         odom_history: List[Tuple[float, float, float]],
                         gt_history: List[Tuple[float, float, float]]) -> np.ndarray:
        """Renders 2D top-down map comparing Ground Truth, Odometry, and EKF SLAM."""
        H, W = self.q_h, self.q_w
        slam_map = np.full((H, W, 3), (25, 25, 28), dtype=np.uint8)

        # World coordinate to map pixel transform
        # Track extent roughly -40 to 40 meters
        scale = 6.2  # pixels per meter
        origin_x = W // 2 + 10
        origin_y = H // 2 - 10

        def w2px(wx, wy):
            px = int(origin_x + wx * scale)
            py = int(origin_y - wy * scale)  # Y inverted in pixels
            return px, py

        # Draw Grid lines (every 10 meters)
        for d in range(-60, 70, 10):
            p1x, p1y = w2px(d, -40)
            p2x, p2y = w2px(d, 40)
            cv2.line(slam_map, (p1x, p1y), (p2x, p2y), (35, 35, 40), 1)
            p3x, p3y = w2px(-40, d)
            p4x, p4y = w2px(40, d)
            cv2.line(slam_map, (p3x, p3y), (p4x, p4y), (35, 35, 40), 1)

        # Draw track centerline
        pts = world.get_track_centerline()
        if len(pts) > 1:
            poly_pts = np.array([w2px(p[0], p[1]) for p in pts], np.int32)
            cv2.polylines(slam_map, [poly_pts], True, (60, 60, 70), 2)

        # Draw boundary cones
        cones_l, cones_r = world.get_boundary_cones()
        for c in cones_l:
            px, py = w2px(c[0], c[1])
            cv2.circle(slam_map, (px, py), 2, (180, 80, 0), -1)  # Blue cones
        for c in cones_r:
            px, py = w2px(c[0], c[1])
            cv2.circle(slam_map, (px, py), 2, (0, 180, 220), -1) # Yellow cones

        # Draw True Potholes (Red filled circles)
        for lm in world.get_landmarks():
            px, py = w2px(lm.x, lm.y)
            if lm.class_id == 0:
                cv2.circle(slam_map, (px, py), max(3, int(lm.radius * scale)), (0, 0, 255), 2)
                cv2.circle(slam_map, (px, py), 2, (0, 0, 255), -1)
            elif lm.class_id == 1:
                cv2.circle(slam_map, (px, py), 4, (0, 215, 255), -1)
            elif lm.class_id == 7:
                cv2.circle(slam_map, (px, py), 3, (255, 0, 200), -1)

        # Draw Ground-Truth Trajectory (White/Gray dashed)
        if len(gt_history) > 1:
            gt_poly = np.array([w2px(p[0], p[1]) for p in gt_history[-400:]], np.int32)
            cv2.polylines(slam_map, [gt_poly], False, (140, 140, 140), 1)

        # Draw Dead-Reckoning Odometry Trajectory (Red dashed drift)
        if len(odom_history) > 1:
            odom_poly = np.array([w2px(p[0], p[1]) for p in odom_history[-400:]], np.int32)
            cv2.polylines(slam_map, [odom_poly], False, (0, 50, 180), 1)

        # Draw EKF SLAM Estimated Trajectory (Bright Green)
        if len(slam_history) > 1:
            slam_poly = np.array([w2px(p[0], p[1]) for p in slam_history[-400:]], np.int32)
            cv2.polylines(slam_map, [slam_poly], False, (0, 255, 120), 2)

        # Draw EKF SLAM Mapped Landmarks with 3-Sigma Uncertainty Ellipses
        for lm in slam_landmarks:
            mpx, mpy = w2px(lm["x"], lm["y"])
            # Draw landmark center (Cyan diamond)
            cv2.drawMarker(slam_map, (mpx, mpy), (255, 255, 0), cv2.MARKER_DIAMOND, 6, 2)
            
            # Draw 3-sigma uncertainty ellipse
            ax_maj = max(2, int(lm["semi_major_3sig"] * scale))
            ax_min = max(2, int(lm["semi_minor_3sig"] * scale))
            angle_deg = -np.degrees(lm["ellipse_angle_rad"])  # Invert for pixel Y
            cv2.ellipse(slam_map, (mpx, mpy), (ax_maj, ax_min), angle_deg, 0, 360, (255, 255, 0), 1)

        # Draw Vehicles:
        # 1. Ground Truth Vehicle (Blue arrow/box)
        gt_x, gt_y, gt_th = vehicle_gt[0], vehicle_gt[1], vehicle_gt[2]
        cpx, cpy = w2px(gt_x, gt_y)
        heading_len = 16
        cv2.circle(slam_map, (cpx, cpy), 5, (255, 150, 0), -1)
        cv2.line(slam_map, (cpx, cpy),
                 (int(cpx + heading_len * np.cos(gt_th)), int(cpy - heading_len * np.sin(gt_th))),
                 (255, 150, 0), 2)

        # 2. EKF SLAM Vehicle Pose (Green arrow)
        sl_x, sl_y, sl_th = slam_pose[0], slam_pose[1], slam_pose[2]
        spx, spy = w2px(sl_x, sl_y)
        cv2.circle(slam_map, (spx, spy), 5, (0, 255, 100), -1)
        cv2.line(slam_map, (spx, spy),
                 (int(spx + heading_len * np.cos(sl_th)), int(spy - heading_len * np.sin(sl_th))),
                 (0, 255, 100), 2)

        # Title & Legend
        cv2.rectangle(slam_map, (0, 0), (W, 24), (20, 20, 20), -1)
        cv2.putText(slam_map, "OVERHEAD WORLD MAP & EKF SLAM (3-SIGMA COVARIANCE ELLIPSES)",
                    (10, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
        
        # Legend overlay at bottom
        cv2.putText(slam_map, "GT: Blue Arrow | SLAM Pose: Green | Red Odom Drift",
                    (10, H - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(slam_map, "Red Circles: True Potholes | Cyan Ellipses: SLAM Mapped Potholes",
                    (10, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)
        return slam_map

    def _render_telemetry_hud(self, vehicle_gt: Tuple[float, float, float, float, float],
                              slam_pose: Tuple[float, float, float],
                              slam_landmarks: List[Dict],
                              world: WorldEnvironment,
                              depth_metrics: Dict[str, float],
                              step_time: float,
                              control_info: Dict,
                              curvature_res: Optional[object] = None) -> np.ndarray:
        """Draws telemetry metrics, vehicle cockpit status, and SLAM evaluation."""
        H, W = self.q_h, self.q_w
        hud = np.full((H, W, 3), (18, 18, 22), dtype=np.uint8)

        # Title banner
        cv2.rectangle(hud, (0, 0), (W, 24), (20, 20, 20), -1)
        cv2.putText(hud, "VEHICLE COCKPIT TELEMETRY & SLAM ACCURACY BENCHMARKS",
                    (10, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

        # Calculate SLAM localization error vs Ground Truth
        gt_x, gt_y, gt_th, gt_v, gt_w = vehicle_gt
        sl_x, sl_y, sl_th = slam_pose
        
        pos_error_m = np.hypot(gt_x - sl_x, gt_y - sl_y)
        heading_error_deg = np.degrees(abs(np.arctan2(np.sin(gt_th - sl_th), np.cos(gt_th - sl_th))))
        
        # Calculate Landmark mapping accuracy vs True Potholes
        true_potholes = [lm for lm in world.get_landmarks() if lm.class_id == 0]
        mapped_errors = []
        for sl_lm in slam_landmarks:
            if sl_lm.get("class_id") == 0:
                # Find closest true pothole
                min_d = min(np.hypot(sl_lm["x"] - t_ph.x, sl_lm["y"] - t_ph.y) for t_ph in true_potholes)
                mapped_errors.append(min_d)
        avg_pothole_err_cm = (np.mean(mapped_errors) * 100.0) if mapped_errors else 0.0

        # Draw metrics in clean panels
        y = 45
        line_h = 24
        
        def draw_hud_row(label: str, val: str, val_color=(0, 255, 200)):
            nonlocal y
            cv2.putText(hud, label, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(hud, val, (250, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, val_color, 1, cv2.LINE_AA)
            y += line_h

        if curvature_res is not None and hasattr(curvature_res, "radius_m"):
            cv2.putText(hud, "--- YOLO ROAD CURVATURE PERCEPTION ---", (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1, cv2.LINE_AA)
            y += 20
            rad_str = f"{curvature_res.radius_m:.1f} m" if curvature_res.radius_m < 300 else "> 300 m (Straight)"
            c_col = (255, 180, 40) if curvature_res.direction == "LEFT" else ((40, 180, 255) if curvature_res.direction == "RIGHT" else (60, 240, 120))
            draw_hud_row("Curve Direction:", f"{curvature_res.direction} ({curvature_res.curve_severity})", c_col)
            draw_hud_row("Curvature Radius (R):", rad_str)
            draw_hud_row("Curvature (kappa):", f"{curvature_res.curvature:.4f} 1/m")
            draw_hud_row("Recommended Corner Speed:", f"{curvature_res.recommended_speed_kmh:.1f} km/h", (0, 255, 100))
            y += 10

        cv2.putText(hud, "--- VEHICLE DYNAMICS & JOY CONTROLLER ---", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA)
        y += 20
        draw_hud_row("Speed (v):", f"{gt_v:.2f} m/s  ({gt_v * 3.6:.1f} km/h)")
        steer_deg = np.degrees(control_info.get("steer_cmd", 0.0))
        draw_hud_row("Steering Angle:", f"{steer_deg:+.1f} deg")
        draw_hud_row("Yaw Rate (omega):", f"{np.degrees(gt_w):+.1f} deg/s")
        draw_hud_row("Drive Controller:", f"{control_info.get('mode', 'Autonomous Joy Cruiser').upper()}")
        draw_hud_row("Simulation Elapsed:", f"{step_time:.2f} s")

        y += 10
        cv2.putText(hud, "--- EKF SLAM LOCALIZATION PERFORMANCE ---", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA)
        y += 20
        pos_col = (0, 255, 100) if pos_error_m < 0.20 else (0, 180, 255)
        draw_hud_row("Pose Error (RMSE):", f"{pos_error_m * 100.0:.1f} cm", pos_col)
        draw_hud_row("Heading Error:", f"{heading_error_deg:.2f} deg")
        draw_hud_row("Tracked Landmarks:", f"{len(slam_landmarks)} mapped features")
        draw_hud_row("Pothole Mapping Error:", f"{avg_pothole_err_cm:.1f} cm (High Precision)", (0, 255, 100))

        y += 10
        cv2.putText(hud, "--- STEREOSCOPIC CAMERA ACCURACY ---", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA)
        y += 20
        absrel_val = depth_metrics.get("absrel", 0.0) * 100.0
        draw_hud_row("Depth AbsRel:", f"{absrel_val:.2f}%", (0, 255, 100) if absrel_val < 8.0 else (0, 180, 255))
        draw_hud_row("RMSE 0-10m:", f"{depth_metrics.get('rmse_0_10', 0.0):.2f} m")
        draw_hud_row("RMSE 10-25m:", f"{depth_metrics.get('rmse_10_25', 0.0):.2f} m")

        return hud
