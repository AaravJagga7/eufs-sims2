"""Third-Person Perspective (TPP) Aerial Drone & Chase Camera Renderer for ADS-DV Racecar.

Renders a high-fidelity 3D aerial drone / chase camera view of the Formula Student
racecar actively navigating the circuit:
1. 3D Racecar Model:
   - Carbon-fiber monocoque chassis in EUFS Navy & Gold racing livery
   - Front wing with dual endplates and nose cone
   - High-downforce dual-element rear wing with yellow endplates
   - Sidepods with aerodynamic air scoops
   - Cockpit opening with driver helmet and safety roll hoop
   - 4 wide Pirelli racing slick tires, with front wheels dynamically steered by delta
   - Glowing rear Formula Student LED rain safety light
2. Environment & Track Ahead:
   - Textured asphalt track surface with perspective lines
   - Inner & outer track boundaries lined with Blue and Yellow Formula Student cones
   - 3D road surface anomalies (potholes with crater depth gradients, speed humps, traffic signs)
   - Glowing road curvature guidance ribbon projected onto the track ahead
3. Broadcast Drone HUD:
   - Digital racing speedometer & gear indicator
   - Steering angle dial
   - Real-time Road Curvature (Radius R, Curvature kappa, Turn Direction)
   - EKF SLAM state status
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np
import cv2

from world_environment import WorldEnvironment, WorldObject
from road_curvature_yolo import CurvatureResult


class TPPCameraModel:
    """Third-Person Perspective (TPP) Aerial Drone & Chase Camera Renderer."""

    def __init__(self, width: int = 640, height: int = 384,
                 chase_dist: float = 4.2,
                 mount_z: float = 2.8,
                 pitch_deg: float = 24.0,
                 fov_deg: float = 70.0):
        self.width = width
        self.height = height
        self.chase_dist = chase_dist    # Distance behind rear axle (m)
        self.mount_z = mount_z          # Height above ground (m)
        self.pitch_rad = math.radians(pitch_deg)  # Downward tilt (rad)
        self.fov_rad = math.radians(fov_deg)

        # Camera Intrinsics
        self.fx = (width * 0.5) / math.tan(self.fov_rad * 0.5)
        self.fy = self.fx
        self.cx = width * 0.5
        self.cy = height * 0.52

        # Vehicle Dimensions for 3D render (ADS-DV Formula Student)
        self.wheelbase = 1.58
        self.track_width = 1.20
        self.tire_radius = 0.23
        self.tire_width = 0.20

        # Background sky & grass
        self._init_background()

    def _init_background(self):
        """Precomputes background sky and grass ground gradients."""
        self.base_sky = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Horizon in drone view (with downward pitch, horizon is near top of screen)
        # phi = -pitch_rad -> v = cy + fy * tan(-pitch_rad)
        horizon_y = int(self.cy - self.fy * math.tan(self.pitch_rad))
        horizon_y = max(15, min(self.height - 40, horizon_y))
        
        # Sky gradient
        for y in range(horizon_y):
            ratio = y / max(1, horizon_y)
            b = int(225 + 25 * ratio)
            g = int(170 + 40 * ratio)
            r = int(110 + 60 * ratio)
            self.base_sky[y, :] = (b, g, r)
            
        # Ground grass gradient (rich racing circuit green)
        for y in range(horizon_y, self.height):
            ratio = (y - horizon_y) / max(1, self.height - horizon_y)
            b = int(32 + 18 * ratio)
            g = int(95 + 30 * ratio)
            r = int(28 + 12 * ratio)
            self.base_sky[y, :] = (b, g, r)

        self.horizon_y = horizon_y

    def project_veh_to_cam(self, x_v: float, y_v: float, z_v: float) -> Tuple[Optional[int], Optional[int], float]:
        """Projects a point in vehicle body frame to Drone camera image coordinates (u, v)."""
        # Vector from camera to target point in vehicle body coordinates:
        dx = x_v - (-self.chase_dist)   # forward distance from camera
        dy = y_v - 0.0                  # lateral distance (positive left)
        dz = z_v - self.mount_z         # vertical distance (negative, since point is below camera)

        # Downward pitch rotation into camera optical frame:
        # Optical axis Z_c tilts down: Z_c = dx * cos(theta) - dz * sin(theta)
        # Down axis Y_c: Y_c = (-dz) * cos(theta) - dx * sin(theta)
        # Right axis X_c: X_c = -dy
        cos_p = math.cos(self.pitch_rad)
        sin_p = math.sin(self.pitch_rad)

        Z_c = dx * cos_p - dz * sin_p
        Y_c = (-dz) * cos_p - dx * sin_p
        X_c = -dy

        if Z_c <= 0.2:
            return None, None, Z_c

        u = int(self.cx + self.fx * X_c / Z_c)
        v = int(self.cy + self.fy * Y_c / Z_c)
        return u, v, Z_c

    def project_world_to_cam(self, x_w: float, y_w: float, z_w: float,
                            car_x: float, car_y: float, car_theta: float) -> Tuple[Optional[int], Optional[int], float]:
        """Projects a world 3D point to Drone camera image coordinates."""
        dx = x_w - car_x
        dy = y_w - car_y
        cos_th = math.cos(car_theta)
        sin_th = math.sin(car_theta)

        # World -> Vehicle Body Frame
        x_v = dx * cos_th + dy * sin_th
        y_v = -dx * sin_th + dy * cos_th
        z_v = z_w

        return self.project_veh_to_cam(x_v, y_v, z_v)

    def render_tpp_view(self, car_x: float, car_y: float, car_theta: float,
                        car_v: float, car_steer: float,
                        world: WorldEnvironment,
                        curvature_res: Optional[CurvatureResult] = None,
                        slam_pose: Optional[Tuple[float, float, float]] = None,
                        controller_mode: str = "Autonomous Joy Cruiser") -> np.ndarray:
        """Renders the full third-person perspective aerial drone frame."""
        frame = self.base_sky.copy()

        # 1. Render Track Asphalt Surface & Road Lines Ahead
        self._render_road_surface(frame, car_x, car_y, car_theta, world)

        # 2. Render Curvature Arc Ribbon on the Road
        if curvature_res is not None and len(curvature_res.bev_centerline) > 2:
            self._render_curvature_ribbon(frame, curvature_res)

        # 3. Render World Landmarks (Potholes, Humps, Traffic Signs)
        self._render_world_objects(frame, car_x, car_y, car_theta, world)

        # 4. Render Boundary Cones
        self._render_cones(frame, car_x, car_y, car_theta, world)

        # 5. Render 3D Racecar Body (Chassis, Wings, Cockpit, Steered Wheels)
        self._render_racecar(frame, car_steer, car_v)

        # 6. Render Modern Aerial Drone HUD
        self._render_hud(frame, car_v, car_steer, curvature_res, slam_pose, controller_mode)

        return frame

    def _render_road_surface(self, frame: np.ndarray,
                             car_x: float, car_y: float, car_theta: float,
                             world: WorldEnvironment):
        """Renders asphalt road corridor with perspective."""
        centerline = world.get_track_centerline()
        half_width = 1.6  # Track half-width (m)

        road_poly_pts_left = []
        road_poly_pts_right = []

        cos_th = math.cos(car_theta)
        sin_th = math.sin(car_theta)

        N = len(centerline)
        for i in range(N):
            pt_c = centerline[i]
            dx = pt_c[0] - car_x
            dy = pt_c[1] - car_y
            x_v = dx * cos_th + dy * sin_th
            y_v = -dx * sin_th + dy * cos_th

            # Render road segments around and ahead of vehicle [-3m, 45m]
            if -3.0 <= x_v <= 45.0 and abs(y_v) <= 22.0:
                next_pt = centerline[(i + 1) % N]
                prev_pt = centerline[(i - 1) % N]
                tx = next_pt[0] - prev_pt[0]
                ty = next_pt[1] - prev_pt[1]
                t_len = math.hypot(tx, ty)
                if t_len > 1e-4:
                    nx = -ty / t_len
                    ny = tx / t_len
                else:
                    nx, ny = 0.0, 1.0

                pt_l = (pt_c[0] + nx * half_width, pt_c[1] + ny * half_width)
                pt_r = (pt_c[0] - nx * half_width, pt_c[1] - ny * half_width)

                u_l, v_l, _ = self.project_world_to_cam(pt_l[0], pt_l[1], 0.0, car_x, car_y, car_theta)
                u_r, v_r, _ = self.project_world_to_cam(pt_r[0], pt_r[1], 0.0, car_x, car_y, car_theta)

                if u_l is not None and u_r is not None:
                    road_poly_pts_left.append((u_l, v_l, x_v))
                    road_poly_pts_right.append((u_r, v_r, x_v))

        if len(road_poly_pts_left) >= 2:
            pts_l = sorted(road_poly_pts_left, key=lambda p: p[2])
            pts_r = sorted(road_poly_pts_right, key=lambda p: p[2])
            
            for j in range(min(len(pts_l), len(pts_r)) - 1):
                poly = np.array([
                    [pts_l[j][0], pts_l[j][1]],
                    [pts_l[j+1][0], pts_l[j+1][1]],
                    [pts_r[j+1][0], pts_r[j+1][1]],
                    [pts_r[j][0], pts_r[j][1]],
                ], dtype=np.int32)
                
                dist_factor = np.clip(pts_l[j][2] / 40.0, 0.0, 1.0)
                asphalt_b = int(48 + 18 * dist_factor)
                asphalt_g = int(50 + 18 * dist_factor)
                asphalt_r = int(52 + 18 * dist_factor)
                cv2.fillPoly(frame, [poly], (asphalt_b, asphalt_g, asphalt_r))

                # Edge rumble strips
                cv2.line(frame, (pts_l[j][0], pts_l[j][1]), (pts_l[j+1][0], pts_l[j+1][1]), (210, 210, 210), 2)
                cv2.line(frame, (pts_r[j][0], pts_r[j][1]), (pts_r[j+1][0], pts_r[j+1][1]), (210, 210, 210), 2)

    def _render_curvature_ribbon(self, frame: np.ndarray, curvature_res: CurvatureResult):
        """Draws glowing road curvature guidance ribbon onto the track surface."""
        pts_left = []
        pts_right = []
        half_w = 0.50

        for pt in curvature_res.bev_centerline:
            x_fwd, y_lat = pt[0], pt[1]
            if x_fwd > 0.5:
                u_l, v_l, _ = self.project_veh_to_cam(x_fwd, y_lat + half_w, 0.02)
                u_r, v_r, _ = self.project_veh_to_cam(x_fwd, y_lat - half_w, 0.02)
                if u_l is not None and u_r is not None:
                    if -50 <= u_l < self.width + 50 and -50 <= v_l < self.height + 50:
                        pts_left.append([u_l, v_l])
                        pts_right.append([u_r, v_r])

        if len(pts_left) >= 2:
            pts_ribbon = np.array(pts_left + pts_right[::-1], dtype=np.int32)
            overlay = frame.copy()
            
            if curvature_res.direction == "LEFT":
                color = (255, 160, 40)
            elif curvature_res.direction == "RIGHT":
                color = (40, 180, 255)
            else:
                color = (60, 240, 120)

            cv2.fillPoly(overlay, [pts_ribbon], color)
            cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    def _render_world_objects(self, frame: np.ndarray,
                              car_x: float, car_y: float, car_theta: float,
                              world: WorldEnvironment):
        """Renders 3D potholes, humps, and signs on the track."""
        for lm in world.get_landmarks():
            u, v, z_cam = self.project_world_to_cam(lm.x, lm.y, 0.0, car_x, car_y, car_theta)
            if u is None or not (-40 <= u < self.width + 40 and -40 <= v < self.height + 40):
                continue
            if z_cam < 0.5 or z_cam > 40.0:
                continue

            scale = max(2.5, self.fx * lm.radius / z_cam)

            if lm.class_id == 0:
                # Pothole: Elliptical crater
                rw = int(scale)
                rh = max(2, int(scale * 0.55))
                cv2.ellipse(frame, (u, v), (rw + 3, rh + 2), 0, 0, 360, (25, 25, 30), -1)
                cv2.ellipse(frame, (u, v), (rw, rh), 0, 0, 360, (10, 10, 12), -1)
                cv2.ellipse(frame, (u, v), (rw + 4, rh + 3), 0, 0, 360, (0, 70, 255), 2)
                
                if z_cam < 20.0:
                    cv2.putText(frame, f"Pothole {lm.depth_m*100:.0f}cm",
                                (u - 30, v - rh - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 200, 255), 1)

            elif lm.class_id == 1:
                # Speed Hump
                rw = int(scale * 1.5)
                rh = max(3, int(scale * 0.35))
                cv2.ellipse(frame, (u, v), (rw, rh), 0, 0, 360, (0, 215, 255), -1)

    def _render_cones(self, frame: np.ndarray,
                      car_x: float, car_y: float, car_theta: float,
                      world: WorldEnvironment):
        """Renders track boundary cones (Yellow right, Blue left)."""
        cones_left, cones_right = world.get_boundary_cones()
        all_cones = []
        for pt in cones_left:
            all_cones.append((pt[0], pt[1], (225, 110, 15)))  # Blue
        for pt in cones_right:
            all_cones.append((pt[0], pt[1], (10, 215, 255)))  # Yellow

        cone_projections = []
        for cx, cy, col in all_cones:
            u, v, z_cam = self.project_world_to_cam(cx, cy, 0.0, car_x, car_y, car_theta)
            if u is not None and -20 <= u < self.width + 20 and -20 <= v < self.height + 20:
                if 1.0 <= z_cam <= 45.0:
                    cone_projections.append((u, v, z_cam, col))

        # Sort far to near
        cone_projections.sort(key=lambda c: c[2], reverse=True)

        for u, v, z_cam, col in cone_projections:
            cone_h = max(4, int(self.fy * 0.35 / z_cam))
            cone_w = max(3, int(cone_h * 0.70))
            pts = np.array([
                [u, v - cone_h],
                [u - cone_w // 2, v],
                [u + cone_w // 2, v]
            ], dtype=np.int32)
            cv2.fillPoly(frame, [pts], col)
            cv2.polylines(frame, [pts], isClosed=True, color=(15, 15, 15), thickness=1)

    def _render_racecar(self, frame: np.ndarray, steer_angle: float, speed_mps: float):
        """Renders the detailed 3D Formula Student ADS-DV racecar."""
        # 1. Rear Diffuser & Rain Safety Light
        rd_u, rd_v, _ = self.project_veh_to_cam(-0.55, 0.0, 0.15)
        if rd_u is not None:
            cv2.rectangle(frame, (rd_u - 24, rd_v - 6), (rd_u + 24, rd_v + 6), (15, 15, 18), -1)
            # Glowing Rain Light
            cv2.circle(frame, (rd_u, rd_v), 7, (0, 0, 255), -1)
            cv2.circle(frame, (rd_u, rd_v), 10, (100, 100, 255), 1)

        # 2. Rear Wing (High downforce dual-element wing)
        rw_tl, rw_tl_v, _ = self.project_veh_to_cam(-0.48, 0.65, 0.88)
        rw_tr, rw_tr_v, _ = self.project_veh_to_cam(-0.48, -0.65, 0.88)
        rw_bl, rw_bl_v, _ = self.project_veh_to_cam(-0.20, 0.65, 0.78)
        rw_br, rw_br_v, _ = self.project_veh_to_cam(-0.20, -0.65, 0.78)

        if all(pt is not None for pt in [rw_tl, rw_tr, rw_bl, rw_br]):
            wing_poly = np.array([[rw_tl, rw_tl_v], [rw_tr, rw_tr_v], [rw_br, rw_br_v], [rw_bl, rw_bl_v]], dtype=np.int32)
            cv2.fillPoly(frame, [wing_poly], (25, 25, 30))
            cv2.polylines(frame, [wing_poly], isClosed=True, color=(0, 215, 255), thickness=2)
            # Endplates
            cv2.line(frame, (rw_tl, rw_tl_v - 15), (rw_bl, rw_bl_v + 15), (0, 220, 255), 4)
            cv2.line(frame, (rw_tr, rw_tr_v - 15), (rw_br, rw_br_v + 15), (0, 220, 255), 4)

        # 3. Rear Wide Racing Slick Tires
        self._render_tire(frame, x_v=0.0, y_v=self.track_width * 0.50, steer=0.0)
        self._render_tire(frame, x_v=0.0, y_v=-self.track_width * 0.50, steer=0.0)

        # 4. Aerodynamic Sidepods
        sp_l1, sp_l1_v, _ = self.project_veh_to_cam(0.20, 0.52, 0.32)
        sp_l2, sp_l2_v, _ = self.project_veh_to_cam(1.05, 0.46, 0.32)
        sp_r1, sp_r1_v, _ = self.project_veh_to_cam(0.20, -0.52, 0.32)
        sp_r2, sp_r2_v, _ = self.project_veh_to_cam(1.05, -0.46, 0.32)
        if all(pt is not None for pt in [sp_l1, sp_l2, sp_r1, sp_r2]):
            cv2.line(frame, (sp_l1, sp_l1_v), (sp_l2, sp_l2_v), (200, 140, 0), 6)
            cv2.line(frame, (sp_r1, sp_r1_v), (sp_r2, sp_r2_v), (200, 140, 0), 6)

        # 5. Main Fuselage / Monocoque Body
        c_rear_l, c_rear_v_l, _ = self.project_veh_to_cam(-0.15, 0.36, 0.44)
        c_rear_r, c_rear_v_r, _ = self.project_veh_to_cam(-0.15, -0.36, 0.44)
        c_mid_l, c_mid_v_l, _ = self.project_veh_to_cam(0.80, 0.34, 0.38)
        c_mid_r, c_mid_v_r, _ = self.project_veh_to_cam(0.80, -0.34, 0.38)
        c_nose_l, c_nose_v_l, _ = self.project_veh_to_cam(1.75, 0.18, 0.26)
        c_nose_r, c_nose_v_r, _ = self.project_veh_to_cam(1.75, -0.18, 0.26)
        c_tip, c_tip_v, _ = self.project_veh_to_cam(2.10, 0.0, 0.20)

        if all(pt is not None for pt in [c_rear_l, c_rear_r, c_mid_l, c_mid_r, c_nose_l, c_nose_r, c_tip]):
            chassis_poly = np.array([
                [c_rear_l, c_rear_v_l],
                [c_mid_l, c_mid_v_l],
                [c_nose_l, c_nose_v_l],
                [c_tip, c_tip_v],
                [c_nose_r, c_nose_v_r],
                [c_mid_r, c_mid_v_r],
                [c_rear_r, c_rear_v_r]
            ], dtype=np.int32)
            
            # EUFS Navy Carbon Fiber Body
            cv2.fillPoly(frame, [chassis_poly], (36, 36, 45))
            cv2.polylines(frame, [chassis_poly], isClosed=True, color=(220, 150, 0), thickness=2)

            # Centerline Gold Racing Stripe
            cs_start, cs_start_v, _ = self.project_veh_to_cam(-0.15, 0.0, 0.45)
            if cs_start is not None and c_tip is not None:
                cv2.line(frame, (cs_start, cs_start_v), (c_tip, c_tip_v), (0, 220, 255), 4)

        # 6. Cockpit Opening, Roll Hoop & Driver Helmet
        rh_top, rh_top_v, _ = self.project_veh_to_cam(0.55, 0.0, 0.98)
        rh_bl, rh_bl_v, _ = self.project_veh_to_cam(0.55, 0.18, 0.52)
        rh_br, rh_br_v, _ = self.project_veh_to_cam(0.55, -0.18, 0.52)
        if all(pt is not None for pt in [rh_top, rh_bl, rh_br]):
            cv2.line(frame, (rh_bl, rh_bl_v), (rh_top, rh_top_v), (210, 210, 215), 5)
            cv2.line(frame, (rh_br, rh_br_v), (rh_top, rh_top_v), (210, 210, 215), 5)

            # Driver Racing Helmet
            h_u, h_v, _ = self.project_veh_to_cam(0.70, 0.0, 0.70)
            if h_u is not None:
                cv2.circle(frame, (h_u, h_v), 14, (0, 215, 255), -1)  # Yellow helmet
                cv2.circle(frame, (h_u, h_v), 14, (20, 20, 20), 2)
                cv2.ellipse(frame, (h_u, h_v - 2), (9, 5), 0, 0, 180, (20, 20, 20), -1)  # Visor

        # 7. Front Steered Tires (Dynamically steered by angle delta!)
        self._render_tire(frame, x_v=self.wheelbase, y_v=self.track_width * 0.48, steer=steer_angle)
        self._render_tire(frame, x_v=self.wheelbase, y_v=-self.track_width * 0.48, steer=steer_angle)

        # 8. Front Wing with Endplates
        fw_l, fw_l_v, _ = self.project_veh_to_cam(2.15, 0.70, 0.16)
        fw_r, fw_r_v, _ = self.project_veh_to_cam(2.15, -0.70, 0.16)
        fw_bl, fw_bl_v, _ = self.project_veh_to_cam(1.95, 0.70, 0.16)
        fw_br, fw_br_v, _ = self.project_veh_to_cam(1.95, -0.70, 0.16)
        if all(pt is not None for pt in [fw_l, fw_r, fw_bl, fw_br]):
            fw_poly = np.array([[fw_l, fw_l_v], [fw_r, fw_r_v], [fw_br, fw_br_v], [fw_bl, fw_bl_v]], dtype=np.int32)
            cv2.fillPoly(frame, [fw_poly], (25, 25, 30))
            cv2.polylines(frame, [fw_poly], isClosed=True, color=(0, 220, 255), thickness=2)
            cv2.line(frame, (fw_l, fw_l_v - 10), (fw_bl, fw_bl_v + 10), (0, 220, 255), 3)
            cv2.line(frame, (fw_r, fw_r_v - 10), (fw_br, fw_br_v + 10), (0, 220, 255), 3)

    def _render_tire(self, frame: np.ndarray, x_v: float, y_v: float, steer: float):
        """Renders a single 3D wheel with steered angle."""
        cos_s = math.cos(steer)
        sin_s = math.sin(steer)
        hw = self.tire_width * 0.5
        hr = self.tire_radius * 0.90

        p_front = (x_v + hr * cos_s, y_v + hr * sin_s, self.tire_radius)
        p_rear = (x_v - hr * cos_s, y_v - hr * sin_s, self.tire_radius)
        p_top = (x_v, y_v, self.tire_radius * 2.0)
        p_bot = (x_v, y_v, 0.0)

        uf, vf, zf = self.project_veh_to_cam(*p_front)
        ur, vr, zr = self.project_veh_to_cam(*p_rear)
        ut, vt, _ = self.project_veh_to_cam(*p_top)
        ub, vb, _ = self.project_veh_to_cam(*p_bot)

        if all(pt is not None for pt in [uf, ur, ut, ub]):
            tire_w = max(7, int(self.fx * self.tire_width / max(1.0, zf)))
            cv2.line(frame, (ur, vr), (uf, vf), (20, 20, 22), tire_w)
            cv2.line(frame, (ub, vb), (ut, vt), (28, 28, 30), tire_w - 2)
            uc, vc, _ = self.project_veh_to_cam(x_v, y_v, self.tire_radius)
            if uc is not None:
                cv2.circle(frame, (uc, vc), max(3, tire_w // 3), (140, 140, 150), -1)

    def _render_hud(self, frame: np.ndarray, speed_mps: float, steer_rad: float,
                    curvature_res: Optional[CurvatureResult],
                    slam_pose: Optional[Tuple[float, float, float]],
                    controller_mode: str):
        """Renders aerial drone telemetry HUD overlay onto the TPP view."""
        H, W = frame.shape[:2]

        # 1. Top Header Banner: EUFS Drone Broadcast HUD
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (W, 40), (16, 16, 22), -1)
        cv2.addWeighted(overlay, 0.82, frame, 0.18, 0, frame)

        # Drone crosshair / REC indicator
        cv2.circle(frame, (18, 20), 5, (0, 0, 255), -1)
        cv2.putText(frame, "AERIAL DRONE CHASE CAM [FPV BROADCAST]",
                    (30, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1, cv2.LINE_AA)

        # 2. Curvature Indicator (Top-Right)
        if curvature_res is not None:
            c_box_w = 230
            c_x1 = W - c_box_w - 10
            overlay = frame.copy()
            cv2.rectangle(overlay, (c_x1, 5), (W - 10, 36), (22, 22, 28), -1)
            cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
            cv2.rectangle(frame, (c_x1, 5), (W - 10, 36), (0, 200, 255), 1)

            if curvature_res.direction == "LEFT":
                d_str, d_col = "<- TURN LEFT", (255, 180, 40)
            elif curvature_res.direction == "RIGHT":
                d_str, d_col = "TURN RIGHT ->", (40, 180, 255)
            else:
                d_str, d_col = "|| STRAIGHT", (60, 255, 120)

            rad_text = f"R={curvature_res.radius_m:.0f}m" if curvature_res.radius_m < 300 else "R > 300m"
            cv2.putText(frame, f"{d_str} | {rad_text}", (c_x1 + 8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, d_col, 1, cv2.LINE_AA)

        # 3. Speedometer & Steer Dial (Bottom-Left)
        g_w, g_h = 220, 72
        gy1 = H - g_h - 10
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, gy1), (10 + g_w, H - 10), (16, 16, 24), -1)
        cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
        cv2.rectangle(frame, (10, gy1), (10 + g_w, H - 10), (0, 220, 255), 1)

        speed_kmh = speed_mps * 3.6
        cv2.putText(frame, f"{speed_kmh:4.1f}", (20, gy1 + 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.05, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "KM/H", (105, gy1 + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)
        
        steer_deg = math.degrees(steer_rad)
        cv2.putText(frame, f"STEER: {steer_deg:+5.1f} deg", (20, gy1 + 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1, cv2.LINE_AA)
