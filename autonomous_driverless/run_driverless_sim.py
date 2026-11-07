"""Main Driverless Simulation Runner.

Integrates:
1. Vehicle Model (EUFS ADS-DV Racecar kinematics and dynamics)
2. World Environment (Asphalt circuit with designated potholes, humps, signs, cones)
3. Custom Stereoscopic Camera Model (ZED baseline, StereoSGBM disparity, 3D metric triangulation)
4. EKF SLAM Model (Continuous localization and landmark mapping with 3-sigma ellipses)
5. YOLO Road Perception Detector (YOLO 26n / 11n or calibrated native detector)
6. Joy Driving Controller (Autonomous joy cruising with active pothole weave avoidance)
7. Multi-View Telemetry Dashboard & Evaluation Metrics Logger
"""

import argparse
import os
import sys
import time
from datetime import date
import numpy as np
import cv2

# Add local path to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vehicle_model import VehicleModel, VehicleParameters
from world_environment import WorldEnvironment
from stereo_camera import StereoCameraModel, StereoCameraCalibration
from ekf_slam import EKFSLAM
from yolo_detector import YOLORoadDetector
from joy_controller import JoyDrivingController
from telemetry_dashboard import TelemetryDashboard
from tpp_camera import TPPCameraModel
from road_curvature_yolo import RoadCurvatureYOLO


def main():
    parser = argparse.ArgumentParser(description="Driverless Vehicle Simulation with Stereo Vision, YOLO & EKF SLAM")
    parser.add_argument("--weights", type=str, default=None, help="Path to trained YOLO weights (e.g. yolo26n.pt, yolo11n.pt)")
    parser.add_argument("--duration", type=float, default=25.0, help="Simulation duration in seconds")
    parser.add_argument("--dt", type=float, default=0.05, help="Simulation time step in seconds (20 Hz)")
    parser.add_argument("--save-video", action="store_true", default=True, help="Save dashboard MP4 video recording")
    parser.add_argument("--save-snapshots", action="store_true", default=True, help="Save key visual snapshot frames")
    parser.add_argument("--metrics-out", type=str, default="results/metrics.csv", help="Metrics CSV output path")
    parser.add_argument("--gui", dest="gui", action="store_true", default=True, help="Display live visual telemetry window on screen")
    parser.add_argument("--no-gui", dest="gui", action="store_false", help="Disable on-screen window")
    args = parser.parse_args()

    print("=================================================================")
    print("   FORMULA STUDENT AUTONOMOUS DRIVING SIMULATION & PERCEPTION   ")
    print("=================================================================")
    print(f"Simulation Duration: {args.duration:.1f} s | dt: {args.dt:.3f} s ({1.0/args.dt:.0f} Hz)")
    print(f"YOLO Weights: {args.weights or 'Native Calibrated Road Perception Detector'}")

    os.makedirs("results", exist_ok=True)
    os.makedirs("logs", exist_ok=True)

    # 1. Initialize World Environment
    print("[1/6] Loading World Environment (Track Circuit, Potholes, Humps, Obstacles)...")
    world = WorldEnvironment(track_type="grand_circuit")
    potholes = [lm for lm in world.get_landmarks() if lm.class_id == 0]
    print(f"      Track generated with {len(world.get_track_centerline())} waypoints.")
    print(f"      Designated {len(potholes)} potholes and {len(world.get_landmarks()) - len(potholes)} road landmarks.")
    for ph in potholes:
        print(f"        * Pothole ID {ph.id} at ({ph.x:+.1f}m, {ph.y:+.1f}m) | radius {ph.radius:.2f}m | depth {ph.depth_m * 100.0:.0f}cm")

    # 2. Initialize Vehicle Model
    print("[2/6] Loading Vehicle Model (ADS-DV Racecar, Wheelbase: 1.58m, ZED Camera Mount)...")
    start_pt = world.get_track_centerline()[0]
    next_pt = world.get_track_centerline()[1]
    init_theta = np.arctan2(next_pt[1] - start_pt[1], next_pt[0] - start_pt[0])
    car = VehicleModel(init_x=start_pt[0], init_y=start_pt[1], init_theta=init_theta)

    # 3. Initialize Stereoscopic Camera Model
    print("[3/6] Initializing Custom Stereoscopic Camera Model (ZED Baseline: 0.12m, 640x384)...")
    calib = StereoCameraCalibration()
    stereo_cam = StereoCameraModel(calib=calib)

    # 4. Initialize EKF SLAM Model
    print("[4/6] Initializing EKF SLAM (State: Pose + Dynamic Pothole/Landmark Map)...")
    slam = EKFSLAM(init_x=start_pt[0], init_y=start_pt[1], init_theta=init_theta)

    # 5. Initialize YOLO Detector
    print("[5/6] Loading YOLO Road Perception Detector...")
    detector = YOLORoadDetector(weights_path=args.weights)

    # 6. Initialize Joy Controller, TPP Chase Camera & Curvature Engine
    print("[6/6] Initializing Joy Controller, Road Curvature YOLO & TPP Camera...")
    joy_ctrl = JoyDrivingController(waypoints=world.get_track_centerline(), wheelbase=car.params.wheelbase)
    dashboard = TelemetryDashboard(width=1280, height=768)
    tpp_cam = TPPCameraModel(width=640, height=384)
    curv_engine = RoadCurvatureYOLO(weights_path=args.weights)

    # Video Writer
    video_writer = None
    video_path = os.path.join("results", "joy_drive_slam_telemetry.mp4")
    if args.save_video:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(video_path, fourcc, int(1.0 / args.dt), (1280, 768))

    # Simulation loop metrics accumulators
    total_steps = int(args.duration / args.dt)
    pose_errors = []
    heading_errors = []
    pothole_mapping_errors = []
    depth_absrel_history = []
    depth_rmse_history = []
    
    window_name = "EUFS Formula Student - Autonomous Driverless Simulation & SLAM"
    if args.gui:
        if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
            os.environ["DISPLAY"] = ":0"
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 1280, 768)
            print("[GUI] Live on-screen telemetry display window active (Press 'q' in window to exit)")
        except Exception as e:
            print(f"[GUI] Display window initialization warning: {e}. Falling back to headless mode.")
            args.gui = False

    print("\n>>> Launching Autonomous Joy Drive & EKF SLAM Loop...")
    sim_time = 0.0
    detected_potholes_for_ctrl = []

    for step in range(total_steps):
        sim_time = step * args.dt
        
        # A. Joy Controller: Calculate steering and speed targets
        target_v, steer_cmd = joy_ctrl.compute_control(
            car_x=car.x, car_y=car.y, car_theta=car.theta,
            car_v=car.v, detected_potholes=detected_potholes_for_ctrl
        )
        car.set_control(throttle_or_v=target_v, steer_delta=steer_cmd, is_target_speed=True)

        # B. Vehicle Model: Step dynamics and get sensor readings
        car.step(dt=args.dt)
        sensors = car.get_sensor_readings()
        noisy_v, noisy_omega = car.get_noisy_odometry_inputs()
        noisy_steer = sensors["steering_angle"]

        # C. EKF SLAM Prediction Step
        slam.predict(v_odom=noisy_v, omega_odom=noisy_omega, dt=args.dt,
                     steer_angle=noisy_steer, wheelbase=car.params.wheelbase, lr=car.params.lr)

        # D. Stereoscopic Camera: Render left, right images and GT depth
        left_img, right_img, depth_gt = stereo_cam.render_stereo_pair(
            car_x=car.x, car_y=car.y, car_theta=car.theta, world=world
        )

        # E. Stereoscopic Depth Estimation: Compute disparity & metric depth map
        disp_pred, depth_pred = stereo_cam.compute_disparity_and_depth(left_img, right_img)
        depth_metrics = stereo_cam.evaluate_depth_metrics(depth_pred, depth_gt)
        depth_absrel_history.append(depth_metrics["absrel"])
        depth_rmse_history.append(depth_metrics["rmse_0_10"])

        # F. YOLO Road Perception: Detect potholes and obstacles in Left camera image
        detections = detector.detect(left_img)

        # G. 3D Triangulation & Feature Localization via Stereo Depth
        triangulated_features = []
        slam_observations = []
        detected_potholes_for_ctrl = []

        for det in detections:
            tri_3d = stereo_cam.triangulate_detection_3d(
                np.array(det["bbox"]), depth_pred, left_img=left_img, right_img=right_img
            )
            triangulated_features.append(tri_3d)
            # Only potholes (class 0) within 10 m feed SLAM — far-range stereo depth is too noisy
            # Humps/signs are used for pothole avoidance control but NOT for EKF state updates
            if tri_3d is not None:
                if det["class_id"] == 0 and tri_3d["range"] <= 10.0:
                    # Physically calibrated stereo range variance: σ² ∝ Z² / (f·b)
                    sig_r = max(0.03, (tri_3d["range"]**2 / (stereo_cam.calib.fx * stereo_cam.calib.baseline)) * 0.12)
                    obs_dict = {
                        "range": tri_3d["range"],
                        "bearing": tri_3d["bearing"],
                        "class_id": det["class_id"],
                        "class_name": det["class_name"],
                        "sigma_range": sig_r
                    }
                    slam_observations.append(obs_dict)
                    detected_potholes_for_ctrl.append(obs_dict)
                elif det["class_id"] == 0 and tri_3d["range"] <= 15.0:
                    # Pothole detected at medium range — only use for avoidance control, not SLAM
                    detected_potholes_for_ctrl.append({
                        "range": tri_3d["range"],
                        "bearing": tri_3d["bearing"],
                        "class_id": det["class_id"],
                        "class_name": det["class_name"],
                    })

        # H. EKF SLAM Update Step: Associate observations and update map & pose
        slam.update(observations=slam_observations, current_time=sim_time)

        # I. Evaluate SLAM Localization Accuracy vs Ground Truth
        sl_x, sl_y, sl_th = slam.get_vehicle_pose()
        pos_err = np.hypot(car.x - sl_x, car.y - sl_y)
        head_err = abs(np.arctan2(np.sin(car.theta - sl_th), np.cos(car.theta - sl_th)))
        pose_errors.append(pos_err)
        heading_errors.append(head_err)

        # Evaluate Landmark Mapping Accuracy
        slam_mapped = slam.get_mapped_landmarks()
        for sl_lm in slam_mapped:
            if sl_lm["class_id"] == 0:
                min_err = min(np.hypot(sl_lm["x"] - ph.x, sl_lm["y"] - ph.y) for ph in potholes)
                pothole_mapping_errors.append(min_err)

        # J. Road Curvature Estimation & Third-Person Perspective (TPP) Camera
        curv_res = curv_engine.estimate_curvature(left_img, depth_pred)
        control_info = {
            "mode": "Pothole Weave Avoidance" if abs(joy_ctrl.avoidance_bias) > 0.01 else "Autonomous Joy Cruiser",
            "steer_cmd": steer_cmd,
            "target_v": target_v
        }
        tpp_img = tpp_cam.render_tpp_view(
            car_x=car.x, car_y=car.y, car_theta=car.theta,
            car_v=car.v, car_steer=car.delta,
            world=world, curvature_res=curv_res,
            slam_pose=(sl_x, sl_y, sl_th),
            controller_mode=control_info["mode"]
        )

        frame = dashboard.render_frame(
            left_img=left_img,
            depth_map=depth_pred,
            detections=detections,
            triangulated_landmarks=triangulated_features,
            vehicle_gt=(car.x, car.y, car.theta, car.v, car.omega),
            vehicle_odom=(car.odom_x, car.odom_y, car.odom_theta),
            slam_pose=(sl_x, sl_y, sl_th),
            slam_landmarks=slam_mapped,
            world=world,
            slam_history=slam.history_pose,
            odom_history=car.trajectory_odom,
            gt_history=car.trajectory_gt,
            depth_metrics=depth_metrics,
            step_time=sim_time,
            control_info=control_info,
            tpp_img=tpp_img,
            curvature_res=curv_res
        )

        if video_writer is not None:
            video_writer.write(frame)

        if args.gui:
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(max(1, int(args.dt * 1000))) & 0xFF
            if key == ord('q'):
                print("\n[User] Simulation interrupted by user keypress in GUI window.")
                break

        # Save snapshot milestones
        if args.save_snapshots and step in [int(total_steps * 0.20), int(total_steps * 0.50), int(total_steps * 0.85)]:
            snap_path = os.path.join("results", f"dashboard_snapshot_step_{step}.jpg")
            cv2.imwrite(snap_path, frame)
            print(f"      [Milestone {step}/{total_steps}] Saved visual snapshot: {snap_path}")

        # Progress log
        if (step + 1) % 100 == 0 or step == total_steps - 1:
            avg_pothole_cm = (np.mean(pothole_mapping_errors[-100:]) * 100.0) if pothole_mapping_errors else 0.0
            rad_info = f"R={curv_res.radius_m:.0f}m ({curv_res.direction})" if curv_res.radius_m < 300 else "Straight"
            print(f"  Step {step+1:4d}/{total_steps} (t={sim_time:.1f}s) | "
                  f"Speed: {car.v*3.6:4.1f} km/h | "
                  f"Curve: {rad_info:14s} | "
                  f"Pose RMSE: {pos_err*100.0:4.1f} cm | "
                  f"Potholes: {len(slam_mapped)}/7 | "
                  f"Pothole Acc: {avg_pothole_cm:4.1f} cm")

    if args.gui:
        cv2.destroyAllWindows()

    if video_writer is not None:
        video_writer.release()
        print(f"\n[Video] Successfully recorded multi-view telemetry video to: {video_path}")

    # Summary Statistics
    final_rmse_pos = float(np.sqrt(np.mean(np.array(pose_errors)**2)))
    final_rmse_heading = float(np.degrees(np.sqrt(np.mean(np.array(heading_errors)**2))))
    final_pothole_err = float(np.mean(pothole_mapping_errors) * 100.0) if pothole_mapping_errors else 0.0
    final_absrel = float(np.mean(depth_absrel_history) * 100.0)
    final_depth_rmse = float(np.mean(depth_rmse_history))

    print("\n=================================================================")
    print("                 FINAL PERFORMANCE BENCHMARKS                   ")
    print("=================================================================")
    print(f"EKF SLAM Vehicle Position RMSE:       {final_rmse_pos * 100.0:6.2f} cm")
    print(f"EKF SLAM Vehicle Heading RMSE:        {final_rmse_heading:6.2f} deg")
    print(f"Discovered & Mapped Potholes:         {len(slam.get_mapped_landmarks())} / {len(potholes)}")
    print(f"Pothole 3D Localization Accuracy:     {final_pothole_err:6.2f} cm")
    print(f"Stereoscopic Camera Depth AbsRel:     {final_absrel:6.2f} %")
    print(f"Stereoscopic Camera RMSE (0-10m):     {final_depth_rmse:6.2f} m")
    print("=================================================================")

    # Append to metrics.csv conforming to road_perception schema:
    # date,run_name,model,dataset,split,metric_name,value
    today_str = date.today().isoformat()
    metrics_records = [
        (today_str, "autonomous_joy_drive", "stereo_ekf_slam", "grand_circuit", "sim", "slam_pos_rmse_m", f"{final_rmse_pos:.4f}"),
        (today_str, "autonomous_joy_drive", "stereo_ekf_slam", "grand_circuit", "sim", "slam_heading_rmse_deg", f"{final_rmse_heading:.4f}"),
        (today_str, "autonomous_joy_drive", "stereo_ekf_slam", "grand_circuit", "sim", "pothole_map_error_cm", f"{final_pothole_err:.4f}"),
        (today_str, "autonomous_joy_drive", "stereo_cam_zed", "grand_circuit", "sim", "depth_absrel", f"{final_absrel/100.0:.4f}"),
        (today_str, "autonomous_joy_drive", "stereo_cam_zed", "grand_circuit", "sim", "depth_rmse_0_10", f"{final_depth_rmse:.4f}"),
    ]
    
    csv_file = args.metrics_out
    write_header = not os.path.exists(csv_file) or os.path.getsize(csv_file) == 0
    with open(csv_file, "a") as f:
        if write_header:
            f.write("date,run_name,model,dataset,split,metric_name,value\n")
        for rec in metrics_records:
            f.write(",".join(rec) + "\n")
    print(f"[Metrics] Appended benchmark metrics to: {csv_file}")


if __name__ == "__main__":
    main()
