"""Stereo Dataset Generator Module.

Generates a synthetic yet photorealistic stereoscopic perception dataset
directly from the vehicle's onboard ZED stereo camera:
- Conforms strictly to road_perception/datasets/contract.py:
    * Resolution: 640x384 (IMG_W, IMG_H)
    * Left images: data_processed/car_stereo/images/{split}/{stem}.jpg
    * Right images: data_processed/car_stereo/images_right/{split}/{stem}.jpg
    * Depth maps: data_processed/car_stereo/depth/{split}/{stem}.png (16-bit uint, metres * 256)
    * Detection labels: data_processed/car_stereo/labels_det/{split}/{stem}.txt (YOLO cx cy w h)
    * Calibrations: data_processed/car_stereo/calib/{split}/{stem}.json
- Classes:
    * 0: pothole
    * 1: hump
    * 7: traffic_sign
"""

import json
import os
import cv2
import numpy as np

from vehicle_model import VehicleModel
from world_environment import WorldEnvironment
from stereo_camera import StereoCameraModel, StereoCameraCalibration
from joy_controller import JoyDrivingController


def generate_stereo_dataset(num_samples: int = 250, output_dir: str = "data_processed/car_stereo"):
    print(f"\n[Dataset] Generating {num_samples} calibrated stereo camera pairs in: {output_dir}")
    
    splits = {"train": int(0.70 * num_samples), "val": int(0.15 * num_samples), "test": int(0.15 * num_samples)}
    # Ensure sum matches
    splits["test"] = num_samples - splits["train"] - splits["val"]
    
    for split in splits.keys():
        os.makedirs(os.path.join(output_dir, "images", split), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images_right", split), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "depth", split), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels_det", split), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "calib", split), exist_ok=True)

    world = WorldEnvironment(track_type="grand_circuit")
    start_pt = world.get_track_centerline()[0]
    next_pt = world.get_track_centerline()[1]
    init_theta = np.arctan2(next_pt[1] - start_pt[1], next_pt[0] - start_pt[0])
    car = VehicleModel(init_x=start_pt[0], init_y=start_pt[1], init_theta=init_theta)
    
    calib = StereoCameraCalibration()
    stereo_cam = StereoCameraModel(calib=calib)
    joy_ctrl = JoyDrivingController(waypoints=world.get_track_centerline(), wheelbase=car.params.wheelbase)

    split_list = (["train"] * splits["train"]) + (["val"] * splits["val"]) + (["test"] * splits["test"])
    stems_by_split = {"train": [], "val": [], "test": []}

    dt = 0.05
    step = 0
    saved_count = 0

    while saved_count < num_samples:
        step += 1
        # Drive car
        target_v, steer_cmd = joy_ctrl.compute_control(car.x, car.y, car.theta, car.v)
        car.set_control(target_v, steer_cmd)
        car.step(dt)

        # Collect sample every 2 steps
        if step % 2 != 0:
            continue

        split = split_list[saved_count]
        stem = f"stereo_{split}_{saved_count:05d}"
        stems_by_split[split].append(stem)

        # Render Left, Right and GT Depth
        left_img, right_img, depth_gt = stereo_cam.render_stereo_pair(car.x, car.y, car.theta, world)

        # Find visible landmarks in Left image to write YOLO labels
        visible_lms = world.get_visible_landmarks(car.x, car.y, car.theta, max_range=35.0)
        yolo_labels = []

        for lm in visible_lms:
            Xc, Yc, Zc = stereo_cam._world_to_camera(lm.x, lm.y, lm.z, car.x, car.y, car.theta)
            if Zc < 0.8 or Zc > 35.0:
                continue
            u = calib.fx * (Xc / Zc) + calib.cx
            v = calib.fy * (Yc / Zc) + calib.cy
            
            rx = calib.fx * (lm.radius / Zc)
            ry = calib.fy * (lm.radius / Zc) * 0.45 if lm.class_id in [0, 1] else calib.fy * (lm.radius / Zc)
            
            # Normalized YOLO coordinates: class cx cy w h
            norm_cx = u / calib.width
            norm_cy = v / calib.height
            norm_w = (rx * 2.0) / calib.width
            norm_h = (ry * 2.0) / calib.height

            # Clip within image
            if 0.0 < norm_cx < 1.0 and 0.0 < norm_cy < 1.0:
                norm_w = min(norm_w, 1.0)
                norm_h = min(norm_h, 1.0)
                yolo_labels.append(f"{lm.class_id} {norm_cx:.6f} {norm_cy:.6f} {norm_w:.6f} {norm_h:.6f}")

        # Save files
        left_path = os.path.join(output_dir, "images", split, f"{stem}.jpg")
        right_path = os.path.join(output_dir, "images_right", split, f"{stem}.jpg")
        depth_path = os.path.join(output_dir, "depth", split, f"{stem}.png")
        label_path = os.path.join(output_dir, "labels_det", split, f"{stem}.txt")
        calib_path = os.path.join(output_dir, "calib", split, f"{stem}.json")

        cv2.imwrite(left_path, left_img)
        cv2.imwrite(right_path, right_img)
        
        # 16-bit PNG depth: metres * 256.0
        depth_uint16 = (depth_gt * 256.0).clip(0, 65535).astype(np.uint16)
        cv2.imwrite(depth_path, depth_uint16)

        with open(label_path, "w") as f:
            f.write("\n".join(yolo_labels) + "\n")

        calib_meta = {
            "baseline": calib.baseline,
            "fx": calib.fx,
            "fy": calib.fy,
            "cx": calib.cx,
            "cy": calib.cy,
            "fb": calib.fx * calib.baseline,
            "scale": 1.0
        }
        with open(calib_path, "w") as f:
            json.dump(calib_meta, f, indent=2)

        saved_count += 1
        if saved_count % 50 == 0:
            print(f"  Processed {saved_count}/{num_samples} stereo pairs...")

    # Save splits.json
    with open(os.path.join(output_dir, "splits.json"), "w") as f:
        json.dump(stems_by_split, f, indent=2)

    print(f"[Dataset] Completed successfully: {splits['train']} train, {splits['val']} val, {splits['test']} test pairs.")


if __name__ == "__main__":
    generate_stereo_dataset(num_samples=100)
