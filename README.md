# EUFS Sims 2: Complete Autonomous Driverless Formula Student Repository

Comprehensive repository containing the complete Formula Student driverless software stack, simulation environment, perception pipeline, and benchmark logs:

```
.
├── autonomous_driverless/       # Full autonomous driving, vision, SLAM & ROS 2 stack
│   ├── ekf_slam.py              # Extended Kalman Filter SLAM with 3-sigma ellipses
│   ├── road_curvature_yolo.py   # YOLO road perception & analytical curvature engine
│   ├── tpp_camera.py            # Third-person perspective aerial drone chase camera
│   ├── ros_driverless_sim.py    # ROS 2 Humble node publishing sensors & TPP stream
│   ├── launch_ros_sim.launch.py # ROS 2 launch file
│   ├── run_driverless_sim.py    # Standalone simulation runner with live GUI
│   ├── stereo_camera.py         # Custom ZED stereo model & subpixel triangulation
│   ├── vehicle_model.py         # ADS-DV racecar kinematics with slip angle beta
│   ├── world_environment.py     # Grand Circuit track, cones, potholes & humps
│   ├── joy_controller.py        # Pure pursuit with active pothole weave avoidance
│   ├── telemetry_dashboard.py   # Real-time 4-panel HUD & visualization
│   └── data_processed/          # Calibrated car stereo dataset with YOLO annotations
├── eufs_sim2/                   # Official ROS 2 Formula Student simulation framework
│   ├── config/                  # Vehicle parameters and sensor plugin configurations
│   ├── eufs_racecar/            # URDF models, meshes, sensors (ZED, Velodyne) for ADS-DV
│   ├── launch/                  # ROS 2 simulation launch files
│   ├── src/                     # C++ sensor plugins (IMU, wheel speed, cone fusion)
│   └── include/                 # EUFS Sim C++ headers and concepts
├── road_perception/             # Multitask road perception, segmentation & depth models
│   ├── configs/                 # YOLO11, SegFormer & multitask training configs
│   ├── models/                  # Wrappers, loss functions & perception architectures
│   └── scripts/                 # Dataset processing and label generation utilities
├── results/                     # Live simulation video recordings, snapshots & benchmark metrics.csv
└── logs/                        # Simulation run logs
```

---

## Key Capabilities & Benchmarks

1. **YOLO Road Curvature Estimation**:
   - Inverse Perspective Mapping (IPM) & 2nd-order polynomial curve fitting: $Y = aX^2 + bX + c$.
   - Analytical curvature $\kappa = \frac{|2a|}{(1 + b^2)^{3/2}}$, radius $R = 1/\kappa$, and dynamic cornering speed profiling.
2. **Third-Person Perspective (TPP) Aerial Drone Shot**:
   - Elevated chase camera rig ($4.2\,\text{m}$ behind, $2.8\,\text{m}$ altitude, pitched down $24^\circ$).
   - Dynamically steered front wheels matching actual steering angle commands ($\delta$).
   - Full 3D Formula Student ADS-DV racecar chassis in EUFS navy/yellow racing livery with driver helmet.
3. **EKF SLAM with 3-Sigma Confidence Ellipses**:
   - State vector: $\mathbf{x} = [x_v, y_v, \theta_v, m_{1x}, m_{1y}, \dots, m_{Nx}, m_{Ny}]^T$.
   - 4-hit candidate verification buffer eliminates phantom features.
   - Numerically stable Joseph-form covariance updates.
4. **ROS 2 Humble Node & TFs**:
   - Publishes `/camera/tpp/image_raw`, `/road_curvature`, `/odom`, `/imu/data`, `/camera/left/image_raw`, `/camera/depth`, `/ekf_slam/pose`, `/ekf_slam/landmarks`, and dynamic TF tree (`world` $\to$ `odom` $\to$ `base_footprint` $\to$ `tpp_camera_frame`).

---

## Full Lap Performance Benchmarks (Grand Circuit)

| Benchmark Metric | Measured Result | Performance Standard | Status |
| :--- | :---: | :---: | :---: |
| **EKF SLAM Vehicle Position RMSE** | **69.93 cm** *(9.7 cm at loop closure)* | $< 100\,\text{cm}$ | Passed |
| **EKF SLAM Vehicle Heading RMSE** | **1.72°** | $< 3.0^\circ$ | Passed |
| **Potholes Discovered & Mapped** | **6 / 7** | $\ge 5 / 7$ | Passed |
| **Pothole 3D Localization Accuracy** | **48.49 cm** | $< 100\,\text{cm}$ | Passed |
| **Stereo Depth AbsRel** | **8.62 %** | $< 10.0\%$ | Passed |
| **Stereo Depth RMSE (0–10m)** | **1.89 m** | $< 2.5\,\text{m}$ | Passed |

---

## Quickstart

### Standalone Live On-Screen Simulation
```bash
cd autonomous_driverless
python3 run_driverless_sim.py --duration 35.0 --gui
```

### ROS 2 Humble Simulation
```bash
source /opt/ros/humble/setup.bash
cd autonomous_driverless
python3 ros_driverless_sim.py
```
Or via launch file:
```bash
ros2 launch launch_ros_sim.launch.py rate_hz:=20.0 autonomous_mode:=True
```
