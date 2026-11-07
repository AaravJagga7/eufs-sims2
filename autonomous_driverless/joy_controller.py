"""Joy Controller Module.

Gives the vehicle the "joy of driving around in the world":
1. Autonomous Joy Cruiser:
   - Smooth Pure Pursuit path-following along the race track.
   - Dynamic lookahead and curvature-dependent velocity scheduling.
   - Active Pothole & Hazard Avoidance: seamlessly maneuvers around severe potholes
     while remaining safely within the road corridor.
2. Manual Joypad / Keyboard Teleoperation:
   - Interactive control for throttle, braking, and steering.
"""

from typing import Tuple, List, Optional
import numpy as np


class JoyDrivingController:
    """Intelligent controller for joyful and agile autonomous driving."""

    def __init__(self, waypoints: np.ndarray, wheelbase: float = 1.58):
        self.waypoints = waypoints
        self.wheelbase = wheelbase
        self.mode = "autonomous"  # "autonomous" or "manual"
        
        # Pure pursuit tuning parameters
        self.k_lookahead = 0.85
        self.min_lookahead = 3.2
        self.max_lookahead = 8.0
        self.target_cruise_speed = 7.5  # m/s (~27 km/h)
        self.corner_speed = 4.2         # m/s
        
        # Pothole avoidance state
        self.avoidance_bias = 0.0
        self.avoidance_decay = 0.92

    def set_mode(self, mode: str):
        """Switch between 'autonomous' and 'manual'."""
        self.mode = mode

    def compute_control(self, car_x: float, car_y: float, car_theta: float,
                        car_v: float, detected_potholes: Optional[List] = None) -> Tuple[float, float]:
        """Calculates throttle/speed command and steering angle.
        
        Returns:
            target_v: target speed in m/s
            steering_delta: steering angle in radians
        """
        if self.mode == "autonomous":
            return self._autonomous_joy_cruise(car_x, car_y, car_theta, car_v, detected_potholes)
        else:
            return self.target_cruise_speed, 0.0

    def _autonomous_joy_cruise(self, car_x: float, car_y: float, car_theta: float,
                               car_v: float, detected_potholes: Optional[List] = None) -> Tuple[float, float]:
        """Smooth pure pursuit path tracking with pothole avoidance."""
        # 1. Calculate lookahead distance
        lookahead = np.clip(self.k_lookahead * max(car_v, 2.0), self.min_lookahead, self.max_lookahead)
        
        # 2. Find closest waypoint
        dists = np.hypot(self.waypoints[:, 0] - car_x, self.waypoints[:, 1] - car_y)
        closest_idx = int(np.argmin(dists))
        n_pts = len(self.waypoints)
        
        # 3. Find lookahead target waypoint along track forward direction
        target_idx = closest_idx
        for i in range(1, n_pts):
            idx = (closest_idx + i) % n_pts
            d = np.hypot(self.waypoints[idx, 0] - car_x, self.waypoints[idx, 1] - car_y)
            if d >= lookahead:
                target_idx = idx
                break
                
        target_pt = self.waypoints[target_idx]
        
        # 4. Angle to target relative to vehicle heading
        dx = target_pt[0] - car_x
        dy = target_pt[1] - car_y
        angle_to_target = np.arctan2(dy, dx)
        alpha = np.arctan2(np.sin(angle_to_target - car_theta), np.cos(angle_to_target - car_theta))
        
        # Pure pursuit base steering angle: delta = atan2(2 * L * sin(alpha), L_d)
        base_steer = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), lookahead)
        
        # 5. Active Pothole Avoidance Maneuver
        if detected_potholes:
            for ph in detected_potholes:
                # Range and bearing to pothole
                r = ph.get("range", 99.0)
                b = ph.get("bearing", 0.0)
                # Check if pothole is right in front of vehicle path
                if 1.8 < r < 6.5 and abs(b) < 0.22:
                    # Steer away from pothole (nudge in opposite direction of bearing)
                    lateral_dodge = -0.16 if b >= 0.0 else 0.16
                    self.avoidance_bias = lateral_dodge
                    break
                    
        # Apply avoidance bias with exponential smoothing decay
        steer_cmd = base_steer + self.avoidance_bias
        self.avoidance_bias *= self.avoidance_decay
        if abs(self.avoidance_bias) < 0.005:
            self.avoidance_bias = 0.0

        # 6. Curvature-based dynamic speed scheduling
        steer_severity = abs(steer_cmd) / 0.45
        target_v = self.target_cruise_speed * (1.0 - 0.55 * min(1.0, steer_severity))
        target_v = max(self.corner_speed, target_v)

        return float(target_v), float(steer_cmd)

    def manual_input(self, throttle_pct: float, steer_pct: float) -> Tuple[float, float]:
        """Processes manual teleoperation command (-1.0 to 1.0)."""
        max_speed = 12.0
        max_steer = 0.45
        target_v = throttle_pct * max_speed
        steer_cmd = steer_pct * max_steer
        return float(target_v), float(steer_cmd)
