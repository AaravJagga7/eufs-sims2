# EUFS Sims 2: Autonomous Formula Student Simulation, Stereo Perception & EKF SLAM

An end-to-end autonomous racing simulation pipeline designed for the **EUFS ADS-DV Formula Student** driverless competition:
- **Vehicle Dynamics Model**: Kinematic bicycle model with dynamic slip angle $\beta$ and realistic sensor noise (wheel encoders, IMU yaw rate & accelerometers).
- **ZED Stereoscopic Camera Model**: Anti-aliased subpixel stereo rendering with epipolar geometry, metric depth disparity matching, and $< 2\,\text{cm}$ near-field triangulation.
- **YOLO Road Perception & Curvature Engine**: Real-time road boundary segmentation, Inverse Perspective Mapping (IPM), 2nd-order polynomial curve fitting, analytical road curvature $\kappa$ and radius $R$, with Formula Student dynamic speed scheduling.
- **Extended Kalman Filter (EKF) SLAM**: Joint state estimation of vehicle pose $[x_v, y_v, \theta_v]$ and 3D landmark locations with 4-hit candidate verification, Joseph-form covariance updates, and $3\sigma$ uncertainty ellipses.
- **Aerial Drone & Chase Camera (TPP View)**: Third-person perspective aerial chase camera rendering the 3D racecar, steered wheels, wings, cones, and road curvature guidance ribbon.
- **ROS 2 Humble Integration**: Full ROS 2 node publishing `/camera/tpp/image_raw`, `/road_curvature`, `/odom`, `/imu/data`, `/camera/left/image_raw`, `/camera/depth`, `/ekf_slam/pose`, `/ekf_slam/landmarks`, and TF transforms.
- **Autonomous Joy Controller**: Pure Pursuit path tracking with curvature-adaptive speed scheduling and real-time pothole weave avoidance.

---

## Architecture Overview

```
                                  +------------------------------+
                                  |      World Environment       |
                                  | (Circuit, Cones, Potholes)   |
                                  +--------------+---------------+
                                                 |
                         +-----------------------+-----------------------+
                         |                                               |
                         v                                               v
        +---------------------------------+             +----------------------------------+
        |   ZED Stereoscopic Camera       |             |   ADS-DV Kinematic Racecar       |
        | (Baseline: 0.12m, 640x384)      |             | (m=230kg, L=1.58m, Slip Beta)    |
        +----------------+----------------+             +----------------+-----------------+
                         |                                               |
             +-----------+-----------+                                   |
             |                       |                                   | Noisy Odom
             v                       v                                   v
+------------------------+  +------------------------+      +---------------------------+
| YOLO Road Perception   |  | StereoSGBM & Parabolic |      | EKF SLAM Predict Step     |
| (Potholes, Humps, Sign)|  | Subpixel Triangulation |      | (Bicycle Kinematics)      |
+------------+-----------+  +-----------+------------+      +-------------+-------------+
             |                          |                                 |
             +------------+-------------+                                 |
                          |                                               |
                          v                                               |
             +----------------------------+                               |
             | 3D Feature Localizer       |                               |
             | (< 2cm near-field accuracy)|                               |
             +------------+---------------+                               |
                          |                                               |
                          v                                               |
             +----------------------------+                               |
             | EKF SLAM Update Step       |<------------------------------+
             | (Joseph Form + 4-Hit Gate) |
             +------------+---------------+
                          |
                          v
             +----------------------------+
             | Corrected State & Map      |
             | (Pose + 3-Sigma Ellipses)  |
             +------------+---------------+
                          |
        +-----------------+-----------------+
        |                                   |
        v                                   v
+-------------------------------+   +-------------------------------+
| Road Curvature YOLO Model     |   | Pure Pursuit & Avoidance      |
| (IPM, Poly Fit, Kappa, Radius)|   | Joy Driving Controller        |
+---------------+---------------+   +---------------+---------------+
                |                                   |
                +-----------------+-----------------+
                                  |
                                  v
                +-----------------------------------+
                | Third-Person Drone Chase Camera   |
                | (3D Car, Steered Wheels, Cones)   |
                +-----------------+-----------------+
                                  |
            +---------------------+---------------------+
            |                                           |
            v                                           v
+------------------------------+            +------------------------------+
| 4-Panel Telemetry Dashboard  |            | ROS 2 Humble Node & TFs      |
| (Live On-Screen Display)     |            | (/odom, /camera/tpp, /curve) |
+------------------------------+            +------------------------------+
```

---

## Directory Structure

```
.
├── ekf_slam.py                  # EKF SLAM state estimator with Joseph form covariance
├── generate_stereo_dataset.py   # Synthesizes 100-sample calibrated car stereo dataset
├── joy_controller.py            # Pure pursuit controller with active pothole weave avoidance
├── launch_ros_sim.launch.py     # ROS 2 launch file for simulation node
├── road_curvature_yolo.py       # YOLO road curvature model, IPM & polynomial fitting
├── ros_driverless_sim.py        # ROS 2 Humble node publishing sensors, SLAM & TPP view
├── run_driverless_sim.py        # Standalone simulation runner with live telemetry GUI
├── stereo_camera.py             # ZED stereo camera geometry & subpixel triangulation
├── telemetry_dashboard.py       # 4-quadrant real-time telemetry HUD renderer
├── tpp_camera.py                # 3D third-person aerial drone chase camera renderer
├── vehicle_model.py             # EUFS ADS-DV vehicle kinematics & noisy odometry
├── world_environment.py         # Grand Circuit layout, boundary cones & designated potholes
├── yolo_detector.py             # YOLO perception with native computer vision fallback
├── data_processed/              # Processed stereo datasets with YOLO annotations
├── results/                     # Telemetry snapshots, metrics.csv, and benchmarks
└── README.md
```

---

## Installation & Requirements

- **OS**: Ubuntu 22.04 LTS (x86_64 or aarch64)
- **Python**: 3.10+
- **Core Dependencies**: `numpy`, `opencv-python`, `scipy`
- **ROS Version** (Optional for ROS mode): ROS 2 Humble

```bash
pip install numpy opencv-python scipy matplotlib
```

---

## Running the Simulation

### 1. Standalone Python Simulation with Live On-Screen GUI
```bash
python3 run_driverless_sim.py --duration 35.0 --gui
```
- Press `q` in the window to pause or exit anytime.
- Use `--no-gui` for headless execution.

### 2. ROS 2 Humble Simulation
```bash
source /opt/ros/humble/setup.bash
python3 ros_driverless_sim.py
```
Or via launch file:
```bash
ros2 launch launch_ros_sim.launch.py rate_hz:=20.0 autonomous_mode:=True
```

### Published ROS 2 Topics
- `/camera/tpp/image_raw` (`sensor_msgs/Image`): Live aerial drone chase camera view.
- `/camera/tpp/camera_info` (`sensor_msgs/CameraInfo`): Intrinsics for drone camera frame.
- `/road_curvature` (`geometry_msgs/Point`): Real-time road geometry ($x = \kappa$, $y = R$, $z = v_{\text{safe}}$).
- `/camera/left/image_raw` & `/camera/depth`: Front ZED stereo images & 32FC1 metric depth.
- `/ekf_slam/pose`: Estimated vehicle pose with full $6\times6$ covariance.
- `/ekf_slam/landmarks`: RViz 3D marker array of mapped potholes with $3\sigma$ ellipsoids.
- `/odom` & `/imu/data`: Odometry & IMU sensor measurements.
- `/cmd_vel`: Autonomous vehicle control inputs.

---

## Benchmarks & Evaluation

Evaluation on the Grand Circuit (156.45m lap):

| Metric | Result | Target |
| :--- | :---: | :---: |
| **Vehicle Position RMSE** | **69.93 cm** | $< 100\,\text{cm}$ |
| **Vehicle Heading RMSE** | **1.72°** | $< 3.0^\circ$ |
| **Pothole Localization Accuracy** | **48.49 cm** | $< 100\,\text{cm}$ |
| **Discovered Potholes** | **6 / 7** | $\ge 5 / 7$ |
| **Stereo Depth AbsRel** | **8.62%** | $< 10.0\%$ |
| **Stereo Depth RMSE (0–10m)** | **1.89 m** | $< 2.5\,\text{m}$ |

---

## License

MIT License. Designed for Formula Student Driverless / AI competitions.

