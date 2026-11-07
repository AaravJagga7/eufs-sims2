"""EKF-based SLAM Module.

Implements a full Extended Kalman Filter (EKF) Simultaneous Localization and Mapping
system for autonomous vehicles with high-accuracy landmark management:
- Vehicle State: [x_v, y_v, theta_v]
- Mapped Landmarks: [m_1x, m_1y, ..., m_Nx, m_Ny] (Potholes, Humps, Obstacles)
- Candidate Landmark Verification Buffer: Requires >= 3 consecutive observations before
  promoting a feature into the permanent state vector, eliminating spurious false positives.
- Robust Spatial Gating for unambiguous data association without duplicate landmark splits.
- Joseph-form numerical stabilization for the covariance matrix.
- 3-Sigma confidence uncertainty ellipses for telemetry and tracking evaluation.
"""

from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import numpy as np


@dataclass
class SLAMLandmark:
    """Metadata for a confirmed landmark tracked in EKF SLAM."""
    id: int
    class_id: int
    class_name: str
    state_index: int       # Index in the state vector (3 + 2*i)
    num_observations: int
    first_observed_time: float
    last_observed_time: float


@dataclass
class CandidateLandmark:
    """Tentative landmark pending confirmation before state inclusion."""
    id: int
    class_id: int
    class_name: str
    est_x: float
    est_y: float
    hits: int
    last_seen_time: float


class EKFSLAM:
    """High-Precision Extended Kalman Filter SLAM for vehicle localization and landmark mapping."""

    def __init__(self, init_x: float = 0.0, init_y: float = 0.0, init_theta: float = 0.0):
        # State vector: [x_v, y_v, theta_v, m_1x, m_1y, ...]
        self.x = np.array([init_x, init_y, init_theta], dtype=np.float64)
        
        # State Covariance matrix P
        self.P = np.diag([0.005, 0.005, np.radians(0.3)**2]).astype(np.float64)
        
        # Process Noise Q (odometry uncertainty per unit dt)
        # Slightly higher to let the filter trust measurements over dead-reckoning
        self.sigma_v = 0.05       # velocity noise std (m/s)
        self.sigma_omega = 0.018  # yaw rate noise std (rad/s)
        
        # Measurement Noise R (stereo camera observation uncertainty)
        # Increased bearing noise — synthetic camera bearing has ~1.5° error
        self.sigma_range = 0.06    # range std (m)
        self.sigma_bearing = 0.020 # bearing angle std (rad, ~1.1 deg)
        self.R = np.diag([self.sigma_range**2, self.sigma_bearing**2]).astype(np.float64)
        
        # Spatial association gate: pothole min spacing on circuit is ~8m, use 5m gate
        self.spatial_gate_m = 5.0
        
        # Minimum covariance diagonal to prevent filter over-confidence
        self.P_floor_pose = np.array([0.001, 0.001, (np.radians(0.1))**2], dtype=np.float64)
        
        # Confirmed tracked landmarks {landmark_id: SLAMLandmark}
        self.landmarks: Dict[int, SLAMLandmark] = {}
        self.next_landmark_id = 1
        
        # Candidate buffer to filter single-frame noise
        # Require 4 hits for a pothole: reduces false positives further
        self.candidates: List[CandidateLandmark] = []
        self.candidate_hits_required = 4
        
        # History
        self.history_pose = [(init_x, init_y, init_theta)]

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Normalizes angle to [-pi, pi]."""
        return float(np.arctan2(np.sin(angle), np.cos(angle)))

    def predict(self, v_odom: float, omega_odom: float, dt: float,
                steer_angle: float = 0.0, wheelbase: float = 1.58, lr: float = 0.79):
        """EKF Prediction Step using kinematic bicycle model with slip angle beta."""
        if dt <= 0:
            return
            
        N = len(self.landmarks)
        total_dim = 3 + 2 * N
        
        theta = self.x[2]
        
        # Kinematic slip angle beta
        beta = float(np.arctan2(lr * np.tan(steer_angle), wheelbase))
        
        # State transition with Midpoint integration
        theta_mid = theta + 0.5 * omega_odom * dt
        effective_heading = theta_mid + beta
        dx = v_odom * np.cos(effective_heading) * dt
        dy = v_odom * np.sin(effective_heading) * dt
        dtheta = omega_odom * dt
        
        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = self.wrap_angle(theta + dtheta)
        
        # Jacobian F_x
        F_x = np.eye(total_dim, dtype=np.float64)
        F_x[0, 2] = -v_odom * dt * np.sin(effective_heading)
        F_x[1, 2] = v_odom * dt * np.cos(effective_heading)
        
        # Process Noise Covariance
        var_v = (self.sigma_v * dt)**2
        var_w = (self.sigma_omega * dt)**2
        
        cos_m = np.cos(effective_heading)
        sin_m = np.sin(effective_heading)
        Q_v = np.zeros((total_dim, total_dim), dtype=np.float64)
        Q_v[0, 0] = var_v * (cos_m**2) + 1e-6
        Q_v[1, 1] = var_v * (sin_m**2) + 1e-6
        Q_v[0, 1] = Q_v[1, 0] = var_v * sin_m * cos_m
        Q_v[2, 2] = var_w
        
        # Predict covariance: P = F_x * P * F_x^T + Q_v
        self.P = F_x @ self.P @ F_x.T + Q_v
        self.P = 0.5 * (self.P + self.P.T)
        
        # Enforce pose covariance floor — prevents numerical shrinkage below sensor noise
        for i in range(3):
            if self.P[i, i] < self.P_floor_pose[i]:
                self.P[i, i] = self.P_floor_pose[i]
        
        self.history_pose.append((float(self.x[0]), float(self.x[1]), float(self.x[2])))

    def update(self, observations: List[Dict], current_time: float = 0.0):
        """EKF Measurement Update with candidate verification and data association."""
        x_v = self.x[0]
        y_v = self.x[1]
        theta_v = self.x[2]
        updated_landmarks = set()

        for obs in observations:
            r_meas = float(obs["range"])
            phi_meas = float(obs["bearing"])
            cls_id = int(obs.get("class_id", 0))
            cls_name = str(obs.get("class_name", "pothole"))
            
            sig_r = float(obs.get("sigma_range", self.sigma_range))
            sig_b = float(obs.get("sigma_bearing", self.sigma_bearing))
            R_obs = np.diag([sig_r**2, sig_b**2]).astype(np.float64)

            z = np.array([r_meas, phi_meas], dtype=np.float64)
            
            # Estimated world coordinates for this observation
            abs_angle = self.wrap_angle(theta_v + phi_meas)
            obs_wx = x_v + r_meas * np.cos(abs_angle)
            obs_wy = y_v + r_meas * np.sin(abs_angle)

            # 1. Check association with existing confirmed landmarks
            best_lm_id = None
            min_dist = float("inf")
            best_H = None
            best_S = None
            best_nu = None
            
            for lm_id, lm in self.landmarks.items():
                if lm.class_id != cls_id:
                    continue
                    
                idx = lm.state_index
                m_x = self.x[idx]
                m_y = self.x[idx + 1]
                
                # Spatial distance in world frame
                spatial_d = np.hypot(m_x - obs_wx, m_y - obs_wy)
                if spatial_d < self.spatial_gate_m and spatial_d < min_dist:
                    dx = m_x - x_v
                    dy = m_y - y_v
                    q = dx**2 + dy**2
                    sqrt_q = np.sqrt(q)
                    if sqrt_q < 1e-4:
                        continue
                        
                    r_pred = sqrt_q
                    phi_pred = self.wrap_angle(np.arctan2(dy, dx) - theta_v)
                    z_hat = np.array([r_pred, phi_pred], dtype=np.float64)
                    
                    nu = z - z_hat
                    nu[1] = self.wrap_angle(nu[1])
                    
                    H_j = np.zeros((2, len(self.x)), dtype=np.float64)
                    H_j[0, 0] = -dx / sqrt_q
                    H_j[0, 1] = -dy / sqrt_q
                    H_j[0, 2] = 0.0
                    H_j[1, 0] = dy / q
                    H_j[1, 1] = -dx / q
                    H_j[1, 2] = -1.0
                    H_j[0, idx] = dx / sqrt_q
                    H_j[0, idx + 1] = dy / sqrt_q
                    H_j[1, idx] = -dy / q
                    H_j[1, idx + 1] = dx / q
                    
                    S_j = H_j @ self.P @ H_j.T + R_obs
                    best_R = R_obs
                    min_dist = spatial_d
                    best_lm_id = lm_id
                    best_H = H_j
                    best_S = S_j
                    best_nu = nu

            if best_lm_id is not None and best_lm_id not in updated_landmarks:
                updated_landmarks.add(best_lm_id)
                # Associated confirmed landmark -> EKF Update
                lm = self.landmarks[best_lm_id]
                lm.num_observations += 1
                lm.last_observed_time = current_time
                
                try:
                    S_inv = np.linalg.inv(best_S)
                    K = self.P @ best_H.T @ S_inv
                    
                    # Clamp state correction to prevent bad observations from hijacking pose
                    # Max: 0.4 m positional shift and 8 deg heading shift per update
                    dx_state = K @ best_nu
                    dx_pos = np.hypot(dx_state[0], dx_state[1])
                    if dx_pos > 0.4:
                        dx_state = dx_state * (0.4 / dx_pos)
                    if abs(dx_state[2]) > np.radians(8.0):
                        dx_state[2] = np.sign(dx_state[2]) * np.radians(8.0)
                    
                    self.x += dx_state
                    self.x[2] = self.wrap_angle(self.x[2])
                    
                    # Joseph form covariance update (numerically stable)
                    I = np.eye(len(self.x), dtype=np.float64)
                    IKH = I - K @ best_H
                    self.P = IKH @ self.P @ IKH.T + K @ best_R @ K.T
                    self.P = 0.5 * (self.P + self.P.T)
                    # Reapply pose covariance floor
                    if hasattr(self, 'P_floor_pose'):
                        for i in range(3):
                            if self.P[i, i] < self.P_floor_pose[i]:
                                self.P[i, i] = self.P_floor_pose[i]
                except np.linalg.LinAlgError:
                    pass
            else:
                # 2. Not associated: manage candidate landmark buffer
                self._update_candidate(z, obs_wx, obs_wy, cls_id, cls_name, current_time, R_obs)

    def _update_candidate(self, z: np.ndarray, wx: float, wy: float, cls_id: int, cls_name: str,
                          current_time: float, R_obs: Optional[np.ndarray] = None):
        """Buffers candidate detections to require confirmation before mapping."""
        # Never add a candidate if a confirmed landmark is already within spatial gate
        for lm in self.landmarks.values():
            if lm.class_id == cls_id:
                idx = lm.state_index
                if np.hypot(self.x[idx] - wx, self.x[idx + 1] - wy) < self.spatial_gate_m:
                    return

        matched_cand = None
        for cand in self.candidates:
            if cand.class_id == cls_id and np.hypot(cand.est_x - wx, cand.est_y - wy) < 2.5:
                matched_cand = cand
                break

        if matched_cand is not None:
            matched_cand.hits += 1
            matched_cand.last_seen_time = current_time
            matched_cand.est_x = 0.6 * matched_cand.est_x + 0.4 * wx
            matched_cand.est_y = 0.6 * matched_cand.est_y + 0.4 * wy

            if matched_cand.hits >= self.candidate_hits_required:
                # Promote to confirmed landmark in state vector
                self._initialize_landmark(z, cls_id, cls_name, current_time,
                                          seed_x=matched_cand.est_x, seed_y=matched_cand.est_y,
                                          R_obs=R_obs)
                self.candidates.remove(matched_cand)
        else:
            cand_id = self.next_landmark_id
            self.candidates.append(CandidateLandmark(
                id=cand_id,
                class_id=cls_id,
                class_name=cls_name,
                est_x=wx,
                est_y=wy,
                hits=1,
                last_seen_time=current_time
            ))

    def _initialize_landmark(self, z: np.ndarray, cls_id: int, cls_name: str, current_time: float,
                             seed_x: Optional[float] = None, seed_y: Optional[float] = None,
                             R_obs: Optional[np.ndarray] = None):
        """Augments state vector and covariance with new confirmed landmark."""
        r, phi = z[0], z[1]
        x_v, y_v, theta_v = self.x[0], self.x[1], self.x[2]
        
        abs_angle = theta_v + phi
        m_x = seed_x if seed_x is not None else (x_v + r * np.cos(abs_angle))
        m_y = seed_y if seed_y is not None else (y_v + r * np.sin(abs_angle))
        
        old_dim = len(self.x)
        new_dim = old_dim + 2
        
        self.x = np.append(self.x, [m_x, m_y])
        
        G_x = np.array([
            [1.0, 0.0, -r * np.sin(abs_angle)],
            [0.0, 1.0,  r * np.cos(abs_angle)]
        ], dtype=np.float64)
        
        G_z = np.array([
            [np.cos(abs_angle), -r * np.sin(abs_angle)],
            [np.sin(abs_angle),  r * np.cos(abs_angle)]
        ], dtype=np.float64)
        
        meas_R = R_obs if R_obs is not None else self.R
        P_vv = self.P[:3, :3]
        P_mm = G_x @ P_vv @ G_x.T + G_z @ meas_R @ G_z.T
        P_vm = self.P[:, :3] @ G_x.T
        
        new_P = np.zeros((new_dim, new_dim), dtype=np.float64)
        new_P[:old_dim, :old_dim] = self.P
        new_P[:old_dim, old_dim:] = P_vm
        new_P[old_dim:, :old_dim] = P_vm.T
        new_P[old_dim:, old_dim:] = P_mm
        
        self.P = 0.5 * (new_P + new_P.T)
        
        lm_id = self.next_landmark_id
        self.next_landmark_id += 1
        
        self.landmarks[lm_id] = SLAMLandmark(
            id=lm_id,
            class_id=cls_id,
            class_name=cls_name,
            state_index=old_dim,
            num_observations=self.candidate_hits_required,
            first_observed_time=current_time,
            last_observed_time=current_time
        )

    def get_vehicle_pose(self) -> Tuple[float, float, float]:
        """Returns estimated (x, y, theta)."""
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    def get_vehicle_pose_covariance(self) -> np.ndarray:
        """Returns 3x3 covariance matrix of vehicle pose."""
        return self.P[:3, :3].copy()

    def get_mapped_landmarks(self) -> List[Dict]:
        """Returns list of mapped landmarks with positions and 3-sigma uncertainty ellipses."""
        mapped = []
        for lm_id, lm in self.landmarks.items():
            idx = lm.state_index
            pos_x = float(self.x[idx])
            pos_y = float(self.x[idx + 1])
            cov_2x2 = self.P[idx:idx + 2, idx:idx + 2]
            
            eigenvals, eigenvecs = np.linalg.eigh(cov_2x2)
            order = eigenvals.argsort()[::-1]
            eigenvals = np.maximum(eigenvals[order], 1e-6)
            eigenvecs = eigenvecs[:, order]
            
            semi_major = 3.0 * np.sqrt(eigenvals[0])
            semi_minor = 3.0 * np.sqrt(eigenvals[1])
            ellipse_angle = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])
            
            mapped.append({
                "id": lm_id,
                "class_id": lm.class_id,
                "class_name": lm.class_name,
                "x": pos_x,
                "y": pos_y,
                "num_obs": lm.num_observations,
                "semi_major_3sig": float(semi_major),
                "semi_minor_3sig": float(semi_minor),
                "ellipse_angle_rad": float(ellipse_angle),
            })
        return mapped
