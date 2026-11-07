#!/usr/bin/env python3
"""ROS 2 Driverless Vehicle Simulation Node.

Integrates:
1. ADS-DV Kinematic Bicycle Vehicle Dynamics
2. World Environment (Circuit, Potholes, Humps, Obstacles)
3. ZED Stereoscopic Camera (Left, Right, 32FC1 Depth & CameraInfo)
4. YOLO Perception & Subpixel 3D Feature Triangulation
5. EKF SLAM (PoseWithCovarianceStamped + MarkerArray of landmarks & 3-sigma ellipses)
6. Autonomous Joy Controller publishing /cmd_vel
7. Full TF2 Coordinate Transform Broadcast:
   world -> odom -> base_footprint -> zed_camera_center -> zed_left_camera_optical_frame
"""

import sys
import os
import math
import numpy as np

# Ensure local modules are accessible
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from cv_bridge import CvBridge

from vehicle_model import VehicleModel
from world_environment import WorldEnvironment
from stereo_camera import StereoCameraModel, StereoCameraCalibration
from ekf_slam import EKFSLAM
from yolo_detector import YOLORoadDetector
from joy_controller import JoyDrivingController
from telemetry_dashboard import TelemetryDashboard
from tpp_camera import TPPCameraModel
from road_curvature_yolo import RoadCurvatureYOLO


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Converts a planar yaw angle (radians) to a ROS Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class DriverlessSimNode(Node):
    """ROS 2 simulation node publishing sensor topics and running EKF SLAM & autonomous control."""

    def __init__(self):
        super().__init__("driverless_sim_node")

        # Declare parameters
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("autonomous_mode", True)
        self.declare_parameter("max_duration_sec", 0.0)  # 0.0 = run indefinitely
        self.declare_parameter("show_gui", True)

        self.rate_hz = self.get_parameter("rate_hz").value
        self.autonomous_mode = self.get_parameter("autonomous_mode").value
        self.max_duration = self.get_parameter("max_duration_sec").value
        self.show_gui = self.get_parameter("show_gui").value
        self.window_name = "EUFS ROS 2 Driverless Sim - Multi-Panel Telemetry"
        if self.show_gui:
            if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
                os.environ["DISPLAY"] = ":0"
            try:
                import cv2
                self.cv2 = cv2
                self.cv2.namedWindow(self.window_name, self.cv2.WINDOW_NORMAL)
                self.cv2.resizeWindow(self.window_name, 1280, 768)
                self.get_logger().info("[GUI] Live on-screen window initialized on screen.")
            except Exception as e:
                self.get_logger().warn(f"[GUI] Failed to initialize GUI window: {e}")
                self.show_gui = False
        self.dt = 1.0 / self.rate_hz

        self.get_logger().info("=====================================================")
        self.get_logger().info("       EUFS DRIVERLESS ROS 2 SIMULATION NODE        ")
        self.get_logger().info(f"   Rate: {self.rate_hz} Hz | Autonomous: {self.autonomous_mode}")
        self.get_logger().info("=====================================================")

        # 1. Initialize simulation sub-components
        self.world = WorldEnvironment(track_type="grand_circuit")
        start_pt = self.world.get_track_centerline()[0]
        next_pt = self.world.get_track_centerline()[1]
        init_theta = math.atan2(next_pt[1] - start_pt[1], next_pt[0] - start_pt[0])

        self.car = VehicleModel(init_x=start_pt[0], init_y=start_pt[1], init_theta=init_theta)
        self.calib = StereoCameraCalibration()
        self.stereo_cam = StereoCameraModel(calib=self.calib)
        self.slam = EKFSLAM(init_x=start_pt[0], init_y=start_pt[1], init_theta=init_theta)
        self.detector = YOLORoadDetector()
        self.joy_ctrl = JoyDrivingController(waypoints=self.world.get_track_centerline(),
                                             wheelbase=self.car.params.wheelbase)
        self.dashboard = TelemetryDashboard(width=1280, height=768)
        self.tpp_cam = TPPCameraModel(width=640, height=384)
        self.curv_engine = RoadCurvatureYOLO()

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # QoS profile for high-rate sensor streams
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", sensor_qos)
        self.pub_img_left = self.create_publisher(Image, "/camera/left/image_raw", sensor_qos)
        self.pub_img_right = self.create_publisher(Image, "/camera/right/image_raw", sensor_qos)
        self.pub_depth = self.create_publisher(Image, "/camera/depth", sensor_qos)
        self.pub_cam_info = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.pub_slam_pose = self.create_publisher(PoseWithCovarianceStamped, "/ekf_slam/pose", 10)
        self.pub_landmarks = self.create_publisher(MarkerArray, "/ekf_slam/landmarks", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        # TPP Chase Camera and Curvature Publishers
        self.pub_img_tpp = self.create_publisher(Image, "/camera/tpp/image_raw", sensor_qos)
        self.pub_tpp_cam_info = self.create_publisher(CameraInfo, "/camera/tpp/camera_info", 10)
        self.pub_curvature = self.create_publisher(Point, "/road_curvature", 10)

        # Subscriber for external control commands
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)

        # External control cache
        self.external_cmd_v = 0.0
        self.external_cmd_steer = 0.0
        self.last_external_cmd_time = self.get_clock().now()

        # Publish static camera transforms once
        self._publish_static_transforms()

        # Step timer
        self.sim_time = 0.0
        self.step_count = 0
        self.detected_potholes = []
        self.timer = self.create_timer(self.dt, self._sim_step)

    def _cmd_vel_callback(self, msg: Twist):
        """Allows external ROS nodes to command vehicle velocity and steering."""
        if not self.autonomous_mode:
            self.external_cmd_v = float(msg.linear.x)
            # Map angular velocity to steering angle (simple approximation: delta = atan(omega * L / v))
            v = max(0.5, abs(self.external_cmd_v))
            self.external_cmd_steer = float(np.clip(
                math.atan2(msg.angular.z * self.car.params.wheelbase, v),
                -self.car.params.max_steer_rad, self.car.params.max_steer_rad
            ))
            self.last_external_cmd_time = self.get_clock().now()

    def _publish_static_transforms(self):
        """Publishes static transform from base_footprint to ZED camera center."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "zed_camera_center"
        t.transform.translation.x = float(self.calib.mount_x)
        t.transform.translation.y = float(self.calib.mount_y)
        t.transform.translation.z = float(self.calib.mount_z)
        
        # Camera pitch
        half_pitch = float(self.calib.mount_pitch_rad) * 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(half_pitch)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(half_pitch)
        
        self.static_tf_broadcaster.sendTransform(t)

        # Static transform: base_footprint -> tpp_camera_frame (chase camera)
        t_tpp = TransformStamped()
        t_tpp.header.stamp = self.get_clock().now().to_msg()
        t_tpp.header.frame_id = "base_footprint"
        t_tpp.child_frame_id = "tpp_camera_frame"
        t_tpp.transform.translation.x = -float(self.tpp_cam.chase_dist)
        t_tpp.transform.translation.y = 0.0
        t_tpp.transform.translation.z = float(self.tpp_cam.mount_z)
        half_pitch_tpp = float(self.tpp_cam.pitch_rad) * 0.5
        t_tpp.transform.rotation.x = 0.0
        t_tpp.transform.rotation.y = math.sin(half_pitch_tpp)
        t_tpp.transform.rotation.z = 0.0
        t_tpp.transform.rotation.w = math.cos(half_pitch_tpp)
        self.static_tf_broadcaster.sendTransform(t_tpp)

    def _sim_step(self):
        """Simulation tick callback."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        self.sim_time += self.dt
        self.step_count += 1

        # 1. Vehicle Control
        if self.autonomous_mode:
            target_v, steer_cmd = self.joy_ctrl.compute_control(
                car_x=self.car.x, car_y=self.car.y, car_theta=self.car.theta,
                car_v=self.car.v, detected_potholes=self.detected_potholes
            )
            self.car.set_control(throttle_or_v=target_v, steer_delta=steer_cmd, is_target_speed=True)
            
            # Publish autonomous command to /cmd_vel
            cmd_msg = Twist()
            cmd_msg.linear.x = float(target_v)
            cmd_msg.angular.z = float(self.car.omega)
            self.pub_cmd_vel.publish(cmd_msg)
        else:
            self.car.set_control(throttle_or_v=self.external_cmd_v, steer_delta=self.external_cmd_steer, is_target_speed=True)

        # 2. Step Vehicle Dynamics
        self.car.step(dt=self.dt)
        noisy_v, noisy_omega = self.car.get_noisy_odometry_inputs()
        sensors = self.car.get_sensor_readings()

        # 3. EKF SLAM Predict
        self.slam.predict(v_odom=noisy_v, omega_odom=noisy_omega, dt=self.dt,
                          steer_angle=sensors["steering_angle"],
                          wheelbase=self.car.params.wheelbase, lr=self.car.params.lr)

        # 4. Stereo Camera Rendering
        left_img, right_img, depth_gt = self.stereo_cam.render_stereo_pair(
            car_x=self.car.x, car_y=self.car.y, car_theta=self.car.theta, world=self.world
        )
        disp_pred, depth_pred = self.stereo_cam.compute_disparity_and_depth(left_img, right_img)

        # 5. YOLO Detection & Stereo Triangulation
        detections = self.detector.detect(left_img)
        slam_observations = []
        self.detected_potholes = []
        triangulated_features = []

        for det in detections:
            tri_3d = self.stereo_cam.triangulate_detection_3d(
                np.array(det["bbox"]), depth_pred, left_img=left_img, right_img=right_img
            )
            triangulated_features.append(tri_3d)
            if tri_3d is not None:
                if det["class_id"] == 0 and tri_3d["range"] <= 10.0:
                    sig_r = max(0.03, (tri_3d["range"]**2 / (self.stereo_cam.calib.fx * self.stereo_cam.calib.baseline)) * 0.12)
                    obs = {
                        "range": tri_3d["range"],
                        "bearing": tri_3d["bearing"],
                        "class_id": det["class_id"],
                        "class_name": det["class_name"],
                        "sigma_range": sig_r
                    }
                    slam_observations.append(obs)
                    self.detected_potholes.append(obs)
                elif det["class_id"] == 0 and tri_3d["range"] <= 15.0:
                    self.detected_potholes.append({
                        "range": tri_3d["range"],
                        "bearing": tri_3d["bearing"],
                        "class_id": det["class_id"]
                    })

        # 6. EKF SLAM Update
        self.slam.update(observations=slam_observations, current_time=self.sim_time)

        # 6b. Road Curvature Estimation & Third-Person Perspective (TPP) Camera
        curv_res = self.curv_engine.estimate_curvature(left_img, depth_pred)
        control_mode = "Pothole Weave Avoidance" if abs(self.joy_ctrl.avoidance_bias) > 0.01 else "Autonomous Joy Cruiser"
        tpp_img = self.tpp_cam.render_tpp_view(
            car_x=self.car.x, car_y=self.car.y, car_theta=self.car.theta,
            car_v=self.car.v, car_steer=self.car.delta,
            world=self.world, curvature_res=curv_res,
            slam_pose=self.slam.get_vehicle_pose(),
            controller_mode=control_mode
        )

        # 7. Publish ROS Topics & TFs
        self._publish_odometry(stamp)
        self._publish_imu(stamp, sensors)
        self._publish_camera_data(stamp, left_img, right_img, depth_pred)
        self._publish_tpp_camera_data(stamp, tpp_img)
        self._publish_curvature_data(stamp, curv_res)
        self._publish_slam_state(stamp)
        self._broadcast_dynamic_tf(stamp)

        # 8. Live On-Screen Telemetry Window
        if self.show_gui:
            sl_x, sl_y, sl_th = self.slam.get_vehicle_pose()
            control_info = {
                "mode": "Pothole Weave Avoidance" if abs(self.joy_ctrl.avoidance_bias) > 0.01 else "Autonomous Joy Cruiser",
                "steer_cmd": self.car.delta,
                "target_v": self.car.v
            }
            depth_metrics = self.stereo_cam.evaluate_depth_metrics(depth_pred, depth_gt)
            frame = self.dashboard.render_frame(
                left_img=left_img,
                depth_map=depth_pred,
                detections=detections,
                triangulated_landmarks=triangulated_features,
                vehicle_gt=(self.car.x, self.car.y, self.car.theta, self.car.v, self.car.omega),
                vehicle_odom=(self.car.odom_x, self.car.odom_y, self.car.odom_theta),
                slam_pose=(sl_x, sl_y, sl_th),
                slam_landmarks=self.slam.get_mapped_landmarks(),
                world=self.world,
                slam_history=self.slam.history_pose,
                odom_history=self.car.trajectory_odom,
                gt_history=self.car.trajectory_gt,
                depth_metrics=depth_metrics,
                step_time=self.sim_time,
                control_info=control_info,
                tpp_img=tpp_img,
                curvature_res=curv_res
            )
            self.cv2.imshow(self.window_name, frame)
            key = self.cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quit requested via GUI window.")
                self.timer.cancel()
                raise SystemExit

        # Logging periodic status
        if self.step_count % (int(self.rate_hz) * 2) == 0:
            sl_x, sl_y, sl_th = self.slam.get_vehicle_pose()
            pos_err_cm = np.hypot(self.car.x - sl_x, self.car.y - sl_y) * 100.0
            n_lms = len(self.slam.get_mapped_landmarks())
            rad_info = f"R={curv_res.radius_m:.0f}m ({curv_res.direction})" if curv_res.radius_m < 300 else "Straight"
            self.get_logger().info(
                f"[t={self.sim_time:5.1f}s] Speed: {self.car.v * 3.6:4.1f} km/h | "
                f"Curve: {rad_info} | "
                f"SLAM Pos Err: {pos_err_cm:5.1f} cm | Mapped Potholes: {n_lms}/7"
            )

        if self.max_duration > 0.0 and self.sim_time >= self.max_duration:
            self.get_logger().info(f"Target duration {self.max_duration:.1f}s reached. Shutting down node.")
            self.timer.cancel()
            raise SystemExit

    def _publish_odometry(self, stamp):
        """Publishes /odom topic."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = float(self.car.odom_x)
        msg.pose.pose.position.y = float(self.car.odom_y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.car.odom_theta)

        msg.twist.twist.linear.x = float(self.car.v)
        msg.twist.twist.angular.z = float(self.car.omega)

        self.pub_odom.publish(msg)

    def _publish_imu(self, stamp, sensors):
        """Publishes /imu/data topic."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_base_link"
        msg.orientation = yaw_to_quaternion(self.car.theta)
        imu_data = sensors.get("imu", {})
        msg.angular_velocity.z = float(imu_data.get("angular_velocity_z", self.car.omega))
        msg.linear_acceleration.x = float(imu_data.get("linear_acceleration_x", 0.0))
        msg.linear_acceleration.y = float(imu_data.get("linear_acceleration_y", 0.0))
        msg.linear_acceleration.z = 9.81
        self.pub_imu.publish(msg)

    def _publish_tpp_camera_data(self, stamp, tpp_img: np.ndarray):
        """Publishes /camera/tpp/image_raw and CameraInfo."""
        msg = self.bridge.cv2_to_imgmsg(tpp_img, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = "tpp_camera_frame"
        self.pub_img_tpp.publish(msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "tpp_camera_frame"
        info.height = int(self.tpp_cam.height)
        info.width = int(self.tpp_cam.width)
        info.distortion_model = "plumb_bob"
        info.k = [self.tpp_cam.fx, 0.0, self.tpp_cam.cx,
                  0.0, self.tpp_cam.fy, self.tpp_cam.cy,
                  0.0, 0.0, 1.0]
        info.p = [self.tpp_cam.fx, 0.0, self.tpp_cam.cx, 0.0,
                  0.0, self.tpp_cam.fy, self.tpp_cam.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub_tpp_cam_info.publish(info)

    def _publish_curvature_data(self, stamp, curv_res):
        """Publishes /road_curvature (x=kappa, y=radius_m, z=recommended_speed_mps)."""
        pt = Point()
        pt.x = float(curv_res.curvature)
        pt.y = float(curv_res.radius_m)
        pt.z = float(curv_res.recommended_speed_mps)
        self.pub_curvature.publish(pt)

    def _publish_camera_data(self, stamp, left_img, right_img, depth_pred):
        """Publishes left image, right image, 32FC1 metric depth, and CameraInfo."""
        # Left Image
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
        left_msg.header.stamp = stamp
        left_msg.header.frame_id = "zed_left_camera_optical_frame"
        self.pub_img_left.publish(left_msg)

        # Right Image
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8")
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = "zed_right_camera_optical_frame"
        self.pub_img_right.publish(right_msg)

        # Depth Map (meters as float32)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_pred.astype(np.float32), encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "zed_left_camera_optical_frame"
        self.pub_depth.publish(depth_msg)

        # CameraInfo
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "zed_left_camera_optical_frame"
        info.height = int(self.calib.height)
        info.width = int(self.calib.width)
        info.distortion_model = "plumb_bob"
        info.k = [self.calib.fx, 0.0, self.calib.cx,
                  0.0, self.calib.fy, self.calib.cy,
                  0.0, 0.0, 1.0]
        info.p = [self.calib.fx, 0.0, self.calib.cx, 0.0,
                  0.0, self.calib.fy, self.calib.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub_cam_info.publish(info)

    def _publish_slam_state(self, stamp):
        """Publishes EKF SLAM pose and 3D visual markers for mapped potholes."""
        sl_x, sl_y, sl_th = self.slam.get_vehicle_pose()
        
        # 1. PoseWithCovarianceStamped
        p_msg = PoseWithCovarianceStamped()
        p_msg.header.stamp = stamp
        p_msg.header.frame_id = "world"
        p_msg.pose.pose.position.x = float(sl_x)
        p_msg.pose.pose.position.y = float(sl_y)
        p_msg.pose.pose.position.z = 0.0
        p_msg.pose.pose.orientation = yaw_to_quaternion(sl_th)
        
        # Fill 6x6 covariance
        cov_6x6 = [0.0] * 36
        cov_6x6[0] = float(self.slam.P[0, 0])
        cov_6x6[1] = float(self.slam.P[0, 1])
        cov_6x6[6] = float(self.slam.P[1, 0])
        cov_6x6[7] = float(self.slam.P[1, 1])
        cov_6x6[35] = float(self.slam.P[2, 2])
        p_msg.pose.covariance = cov_6x6
        self.pub_slam_pose.publish(p_msg)

        # 2. MarkerArray of Mapped Potholes with 3-Sigma Uncertainty Ellipsoids
        marker_array = MarkerArray()
        landmarks = self.slam.get_mapped_landmarks()
        
        for lm in landmarks:
            # Pothole Cylinder Marker
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "world"
            m.ns = "potholes"
            m.id = lm["id"]
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(lm["x"])
            m.pose.position.y = float(lm["y"])
            m.pose.position.z = -0.05
            m.pose.orientation.w = 1.0
            
            # 3-sigma radius or nominal
            scale_x = max(0.5, 3.0 * lm.get("sigma_x", 0.15))
            scale_y = max(0.5, 3.0 * lm.get("sigma_y", 0.15))
            m.scale.x = float(scale_x)
            m.scale.y = float(scale_y)
            m.scale.z = 0.1
            
            m.color.r = 0.95
            m.color.g = 0.20
            m.color.b = 0.20
            m.color.a = 0.85
            marker_array.markers.append(m)

            # Text label marker
            t = Marker()
            t.header.stamp = stamp
            t.header.frame_id = "world"
            t.ns = "pothole_labels"
            t.id = lm["id"] + 1000
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(lm["x"])
            t.pose.position.y = float(lm["y"])
            t.pose.position.z = 0.35
            t.scale.z = 0.3
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 0.9
            t.text = f"Pothole #{lm['id']} ({lm['num_obs']} hits)"
            marker_array.markers.append(t)

        self.pub_landmarks.publish(marker_array)

    def _broadcast_dynamic_tf(self, stamp):
        """Broadcasts dynamic coordinate frames."""
        # 1. world -> odom (correction from SLAM)
        sl_x, sl_y, sl_th = self.slam.get_vehicle_pose()
        
        t_world_odom = TransformStamped()
        t_world_odom.header.stamp = stamp
        t_world_odom.header.frame_id = "world"
        t_world_odom.child_frame_id = "odom"
        t_world_odom.transform.translation.x = float(sl_x - self.car.odom_x)
        t_world_odom.transform.translation.y = float(sl_y - self.car.odom_y)
        t_world_odom.transform.translation.z = 0.0
        t_world_odom.transform.rotation = yaw_to_quaternion(sl_th - self.car.odom_theta)
        self.tf_broadcaster.sendTransform(t_world_odom)

        # 2. odom -> base_footprint (odometry)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_footprint"
        t_odom_base.transform.translation.x = float(self.car.odom_x)
        t_odom_base.transform.translation.y = float(self.car.odom_y)
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation = yaw_to_quaternion(self.car.odom_theta)
        self.tf_broadcaster.sendTransform(t_odom_base)


def main(args=None):
    rclpy.init(args=args)
    node = DriverlessSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
