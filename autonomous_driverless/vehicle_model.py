"""Vehicle Model Module.

Implements a high-fidelity Kinematic and Dynamic Bicycle Model based on the
Formula Student ADS-DV / EUFS vehicle specifications from eufs_sim2.
Includes sensor simulation for wheel encoders, 6-DOF IMU, and steering sensor.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class VehicleParameters:
    """EUFS ADS-DV Racecar physical and kinematic parameters."""
    mass: float = 160.0             # kg (chassis + driverless kit)
    wheelbase: float = 1.580        # meters (L)
    track_width: float = 1.600      # meters (W)
    lf: float = 0.790               # distance from CoG to front axle (m)
    lr: float = 0.790               # distance from CoG to rear axle (m)
    Iz: float = 47.9                # yaw moment of inertia (kg*m^2 from urdf)
    wheel_radius: float = 0.2525    # meters (wheel diameter 0.505 m)
    
    # Actuator limits
    max_steer_rad: float = 0.45     # ~25.8 degrees
    max_steer_rate: float = 1.2     # rad/s
    max_accel: float = 5.0          # m/s^2
    max_brake: float = 8.0          # m/s^2
    max_speed: float = 15.0         # m/s (~54 km/h)
    
    # Sensor noise characteristics (Gaussian 1-sigma)
    wheel_speed_noise_std: float = 0.04    # m/s
    imu_yaw_rate_noise_std: float = 0.015  # rad/s
    imu_accel_noise_std: float = 0.05      # m/s^2
    steer_noise_std: float = 0.005         # rad
    imu_yaw_drift_bias: float = 0.002      # rad/s constant bias


class VehicleModel:
    """Simulates vehicle state evolution and sensor readings."""

    def __init__(self, init_x: float = 0.0, init_y: float = 0.0,
                 init_theta: float = 0.0, params: VehicleParameters = None):
        self.params = params or VehicleParameters()
        
        # Ground-truth state: [x, y, theta, v, delta, omega, a]
        self.x = float(init_x)
        self.y = float(init_y)
        self.theta = float(init_theta)  # heading in radians [-pi, pi]
        self.v = 0.0                    # longitudinal velocity (m/s)
        self.delta = 0.0                # front wheel steering angle (rad)
        self.omega = 0.0                # yaw rate (rad/s)
        self.accel = 0.0                # longitudinal acceleration (m/s^2)
        
        # Actuator command targets
        self.target_v = 0.0
        self.target_delta = 0.0
        
        # Odometry accumulation
        self.odom_x = float(init_x)
        self.odom_y = float(init_y)
        self.odom_theta = float(init_theta)
        
        # History for telemetry
        self.trajectory_gt = [(self.x, self.y, self.theta)]
        self.trajectory_odom = [(self.odom_x, self.odom_y, self.odom_theta)]

    def set_control(self, throttle_or_v: float, steer_delta: float, is_target_speed: bool = True):
        """Set control input (speed target or direct acceleration, and steering angle)."""
        if is_target_speed:
            self.target_v = np.clip(throttle_or_v, -2.0, self.params.max_speed)
        else:
            self.accel = np.clip(throttle_or_v, -self.params.max_brake, self.params.max_accel)
        self.target_delta = np.clip(steer_delta, -self.params.max_steer_rad, self.params.max_steer_rad)

    def step(self, dt: float = 0.05):
        """Advance vehicle state by time dt (seconds) using kinematic bicycle model."""
        # 1. Steering dynamics (rate-limited actuator)
        delta_err = self.target_delta - self.delta
        max_delta_change = self.params.max_steer_rate * dt
        delta_inc = np.clip(delta_err, -max_delta_change, max_delta_change)
        self.delta += delta_inc
        self.delta = float(np.clip(self.delta, -self.params.max_steer_rad, self.params.max_steer_rad))

        # 2. Longitudinal velocity dynamics (closed loop P controller if target_v set)
        v_err = self.target_v - self.v
        kp_v = 2.5
        desired_a = kp_v * v_err
        self.accel = float(np.clip(desired_a, -self.params.max_brake, self.params.max_accel))
        
        self.v += self.accel * dt
        self.v = float(np.clip(self.v, 0.0, self.params.max_speed))

        # 3. Kinematic bicycle model calculations
        # Slip angle beta at center of gravity
        beta = np.arctan2(self.params.lr * np.tan(self.delta), self.params.wheelbase)
        
        # Yaw rate
        if abs(self.v) > 1e-4:
            self.omega = (self.v / self.params.wheelbase) * np.cos(beta) * np.tan(self.delta)
        else:
            self.omega = 0.0

        # State integration with Runge-Kutta 2nd order (Midpoint method)
        theta_mid = self.theta + 0.5 * self.omega * dt
        dx = self.v * np.cos(theta_mid + beta) * dt
        dy = self.v * np.sin(theta_mid + beta) * dt
        dtheta = self.omega * dt

        self.x += dx
        self.y += dy
        self.theta = float(np.arctan2(np.sin(self.theta + dtheta), np.cos(self.theta + dtheta)))

        # 4. Integrate noisy dead reckoning odometry (wheel encoders + IMU + steer)
        noisy_v, noisy_omega = self.get_noisy_odometry_inputs()
        noisy_steer = self.delta + np.random.normal(0.0, self.params.steer_noise_std)
        beta_odom = float(np.arctan2(self.params.lr * np.tan(noisy_steer), self.params.wheelbase))
        odom_mid_theta = self.odom_theta + 0.5 * noisy_omega * dt
        self.odom_x += noisy_v * np.cos(odom_mid_theta + beta_odom) * dt
        self.odom_y += noisy_v * np.sin(odom_mid_theta + beta_odom) * dt
        self.odom_theta = float(np.arctan2(
            np.sin(self.odom_theta + noisy_omega * dt),
            np.cos(self.odom_theta + noisy_omega * dt)
        ))

        self.trajectory_gt.append((self.x, self.y, self.theta))
        self.trajectory_odom.append((self.odom_x, self.odom_y, self.odom_theta))

    def get_ground_truth_pose(self):
        """Returns (x, y, theta, v, omega)."""
        return self.x, self.y, self.theta, self.v, self.omega

    def get_noisy_odometry_inputs(self):
        """Simulates wheel encoder speed and IMU yaw rate with realistic noise."""
        noisy_v = self.v + np.random.normal(0.0, self.params.wheel_speed_noise_std)
        noisy_omega = self.omega + np.random.normal(0.0, self.params.imu_yaw_rate_noise_std) \
            + self.params.imu_yaw_drift_bias
        return float(noisy_v), float(noisy_omega)

    def get_sensor_readings(self):
        """Returns dictionary of sensor measurements for ROS2/telemetry."""
        noisy_v, noisy_omega = self.get_noisy_odometry_inputs()
        noisy_steer = self.delta + np.random.normal(0.0, self.params.steer_noise_std)
        ax = self.accel + np.random.normal(0.0, self.params.imu_accel_noise_std)
        ay = self.v * self.omega + np.random.normal(0.0, self.params.imu_accel_noise_std)
        
        return {
            "wheel_speed": max(0.0, noisy_v),
            "steering_angle": float(noisy_steer),
            "imu": {
                "angular_velocity_z": noisy_omega,
                "linear_acceleration_x": ax,
                "linear_acceleration_y": ay,
            },
            "odometry_pose": (self.odom_x, self.odom_y, self.odom_theta),
            "ground_truth_pose": (self.x, self.y, self.theta)
        }
