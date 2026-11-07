"""World Environment Module.

Defines the autonomous driving track, road geometry, and ground-truth landmarks:
- Potholes (Class 0)
- Speed Bumps / Humps (Class 1)
- Vehicles (Class 2)
- Pedestrians (Class 5)
- Traffic Lights (Class 6)
- Traffic Signs (Class 7)
- Boundary Cones (Blue and Yellow)

Includes synthetic projection and rendering for stereo cameras with exact
depth and disparity physics.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import numpy as np
import cv2


@dataclass
class WorldObject:
    """Represents an object or landmark in the 3D world."""
    id: int
    class_id: int
    class_name: str
    x: float             # center x in world (meters)
    y: float             # center y in world (meters)
    z: float = 0.0       # ground height (meters)
    radius: float = 0.4  # bounding radius (meters)
    depth_m: float = 0.0 # depth feature (for potholes: depression depth e.g. 0.08m)
    length: float = 0.8  # longitudinal dimension (m)
    width: float = 0.8   # lateral dimension (m)
    height: float = 0.4  # vertical dimension (m)
    color_bgr: Tuple[int, int, int] = (40, 40, 40)
    extra_meta: Dict = field(default_factory=dict)


class WorldEnvironment:
    """Simulated race/road environment with asphalt track and landmarks."""

    def __init__(self, track_type: str = "grand_circuit"):
        self.track_type = track_type
        self.waypoints: np.ndarray = np.empty((0, 2))
        self.track_width: float = 4.0  # 4 meters wide road
        
        # World landmark collections
        self.potholes: List[WorldObject] = []
        self.humps: List[WorldObject] = []
        self.obstacles: List[WorldObject] = []
        self.cones_left: List[Tuple[float, float]] = []
        self.cones_right: List[Tuple[float, float]] = []
        self.all_landmarks: List[WorldObject] = []
        
        self._build_circuit()
        self._populate_landmarks()

    def _build_circuit(self):
        """Constructs a smooth, looped driving circuit with varied curvature."""
        # Parametric spline/waypoint loop
        # Loop geometry: Straight -> Curve -> Chicane -> Hairpin -> Straight
        t = np.linspace(0, 2 * np.pi, 200, endpoint=False)
        # Smooth organic racing circuit shape
        scale_x = 28.0
        scale_y = 18.0
        
        # Compound Fourier shape for realistic track layout
        cx = scale_x * (np.cos(t) + 0.25 * np.cos(2 * t))
        cy = scale_y * (np.sin(t) - 0.20 * np.sin(2 * t) + 0.15 * np.cos(3 * t))
        
        # Align starting point near (0, 0) and heading along positive X
        cx = cx - cx[0]
        cy = cy - cy[0]
        
        # Rotate slightly so starting line is clean
        self.waypoints = np.column_stack((cx, cy))
        
        # Calculate track boundaries (left and right cones)
        n_pts = len(self.waypoints)
        for i in range(0, n_pts, 2):  # Every 2 waypoints place cones
            p1 = self.waypoints[i]
            p2 = self.waypoints[(i + 1) % n_pts]
            tangent = p2 - p1
            tangent_len = np.linalg.norm(tangent)
            if tangent_len < 1e-4:
                continue
            unit_tangent = tangent / tangent_len
            # Normal perpendicular vector pointing left
            normal = np.array([-unit_tangent[1], unit_tangent[0]])
            
            half_w = self.track_width * 0.5
            pt_left = p1 + normal * half_w
            pt_right = p1 - normal * half_w
            
            self.cones_left.append((float(pt_left[0]), float(pt_left[1])))
            self.cones_right.append((float(pt_right[0]), float(pt_right[1])))

    def _populate_landmarks(self):
        """Populates designated potholes, humps, and obstacles along the road."""
        obj_id = 1
        
        # 1. Designated Potholes (Class 0: pothole)
        # Placed along the drivable lane at key sectors
        pothole_configs = [
            # (t_index, lane_offset, radius, depth_m)
            (15, 0.4, 0.45, 0.08),    # Sector 1 straight
            (35, -0.6, 0.55, 0.10),   # Sector 1 entry to turn 1
            (60, 0.2, 0.40, 0.07),    # Turn 2 apex
            (95, -0.3, 0.60, 0.12),   # Back straight
            (125, 0.5, 0.50, 0.09),   # Chicane entry
            (155, -0.4, 0.65, 0.11),  # Hairpin approach
            (180, 0.1, 0.45, 0.08),   # Final corner exit
        ]
        
        n_pts = len(self.waypoints)
        for idx, offset, rad, depth in pothole_configs:
            p1 = self.waypoints[idx % n_pts]
            p2 = self.waypoints[(idx + 1) % n_pts]
            tangent = p2 - p1
            unit_t = tangent / (np.linalg.norm(tangent) + 1e-6)
            normal = np.array([-unit_t[1], unit_t[0]])
            
            # Position inside the drivable lane
            pos = p1 + normal * offset
            pothole = WorldObject(
                id=obj_id,
                class_id=0,
                class_name="pothole",
                x=float(pos[0]),
                y=float(pos[1]),
                z=-depth * 0.5,
                radius=rad,
                depth_m=depth,
                length=rad * 2.0,
                width=rad * 2.0,
                height=depth,
                color_bgr=(25, 25, 25),  # Dark weathered pothole core
                extra_meta={"severity": "high" if depth > 0.09 else "moderate"}
            )
            self.potholes.append(pothole)
            self.all_landmarks.append(pothole)
            obj_id += 1

        # 2. Speed Bumps / Humps (Class 1: hump)
        hump_configs = [
            (50, 0.0, 1.6, 0.06),   # Mid-track traffic calming
            (140, 0.0, 1.8, 0.08),  # Approaching turn 4
        ]
        for idx, offset, w, h in hump_configs:
            p1 = self.waypoints[idx % n_pts]
            p2 = self.waypoints[(idx + 1) % n_pts]
            tangent = p2 - p1
            unit_t = tangent / (np.linalg.norm(tangent) + 1e-6)
            normal = np.array([-unit_t[1], unit_t[0]])
            pos = p1 + normal * offset
            
            hump = WorldObject(
                id=obj_id,
                class_id=1,
                class_name="hump",
                x=float(pos[0]),
                y=float(pos[1]),
                z=h * 0.5,
                radius=0.9,
                depth_m=-h,
                length=0.7,
                width=w * 2.0,
                height=h,
                color_bgr=(0, 200, 255),  # Yellow speed bump markings
                extra_meta={"orientation": float(np.arctan2(normal[1], normal[0]))}
            )
            self.humps.append(hump)
            self.all_landmarks.append(hump)
            obj_id += 1

        # 3. Traffic Signs & Test Obstacles (Classes 7: traffic_sign, 2: car)
        # Speed limit sign near start
        p_sign = self.waypoints[8] + np.array([-1.5, 2.5])
        sign_obj = WorldObject(
            id=obj_id,
            class_id=7,
            class_name="traffic_sign",
            x=float(p_sign[0]),
            y=float(p_sign[1]),
            z=1.2,
            radius=0.35,
            length=0.1,
            width=0.6,
            height=1.2,
            color_bgr=(0, 0, 220),
            extra_meta={"type": "speed_limit"}
        )
        self.obstacles.append(sign_obj)
        self.all_landmarks.append(sign_obj)
        obj_id += 1

    def get_track_centerline(self) -> np.ndarray:
        """Returns waypoints array (N, 2)."""
        return self.waypoints

    def get_boundary_cones(self) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        """Returns (cones_left, cones_right)."""
        return self.cones_left, self.cones_right

    def get_landmarks(self) -> List[WorldObject]:
        """Returns all landmarks (potholes, humps, signs, obstacles)."""
        return self.all_landmarks

    def get_visible_landmarks(self, car_x: float, car_y: float, car_theta: float,
                              max_range: float = 30.0, fov_deg: float = 75.0) -> List[WorldObject]:
        """Returns landmarks within the vehicle's sensor range and field-of-view."""
        visible = []
        fov_rad = np.radians(fov_deg)
        half_fov = fov_rad * 0.5
        
        for lm in self.all_landmarks:
            dx = lm.x - car_x
            dy = lm.y - car_y
            dist = np.hypot(dx, dy)
            if dist > max_range or dist < 0.3:
                continue
            
            # Bearing relative to car heading
            angle_to_lm = np.arctan2(dy, dx)
            rel_bearing = np.arctan2(np.sin(angle_to_lm - car_theta), np.cos(angle_to_lm - car_theta))
            
            if abs(rel_bearing) <= half_fov:
                visible.append(lm)
                
        return visible
