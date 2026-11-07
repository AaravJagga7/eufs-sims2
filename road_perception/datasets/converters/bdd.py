"""Converts BDD100K (10K subset) to our shared format.

- Detection categories mapped to classes 2-7:
    2: car
    3: two-wheeler (bicycle, bike, motor, motorcycle)
    4: bus or truck (bus, truck, train)
    5: person (pedestrian, person, rider)
    6: traffic light
    7: traffic sign
- Drivable maps mapped to road mask:
    1 (direct) and 2 (alternative) -> 1 (road)
    0 -> 0 (not road)
    255 -> 255 (ignore)
- Grouped by video ID (prefix before dash in filename) and split into train / val / test.

Run from repo root:
  python -m datasets.converters.bdd --images "<path>\\bdd100k\\10k" --drivable "<path>\\bdd100k_drivable_maps" --labels "<path>\\bdd100k_labels\\100k" --out data_processed/bdd
"""
import argparse
import json
import os
import random
import cv2
import numpy as np

from datasets.contract import SEED
from datasets.converters.common import clean_boxes, make_dirs, save_sample, split_groups, write_splits
from datasets.transforms import letterbox

CATEGORY_MAP = {
    "car": 2,
    "bike": 3,
    "bicycle": 3,
    "motor": 3,
    "motorcycle": 3,
    "bus": 4,
    "truck": 4,
    "train": 4,
    "person": 5,
    "pedestrian": 5,
    "rider": 5,
    "traffic light": 6,
    "traffic sign": 7,
}


def find_file(root_dirs, relative_path):
    """Finds a file across multiple root candidate directories."""
    for root in root_dirs:
        p = os.path.join(root, relative_path)
        if os.path.exists(p):
            return p
    return None


def load_bdd_sample(json_path, img_w=1280.0, img_h=720.0):
    """Parses one BDD annotation JSON file into (classes, boxes)."""
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # In per-image json, objects can be under frames[0]['objects'] or 'labels'
    objects = []
    if "frames" in data and len(data["frames"]) > 0:
        objects = data["frames"][0].get("objects", [])
    elif "labels" in data:
        objects = data.get("labels", [])

    classes = []
    boxes = []
    for obj in objects:
        cat = obj.get("category", "").lower()
        if cat not in CATEGORY_MAP:
            continue
        box2d = obj.get("box2d")
        if not box2d:
            continue

        x1, y1 = float(box2d["x1"]), float(box2d["y1"])
        x2, y2 = float(box2d["x2"]), float(box2d["y2"])

        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0

        classes.append(CATEGORY_MAP[cat])
        boxes.append([cx / img_w, cy / img_h, w / img_w, h / img_h])

    return np.array(classes, dtype=np.int64), np.array(boxes, dtype=np.float32).reshape(-1, 4)


def to_road_mask(drivable_img):
    """Converts BDD drivable map (0: background, 1: direct, 2: alternative) to road mask."""
    road = np.zeros(drivable_img.shape[:2], dtype=np.uint8)
    road[(drivable_img == 1) | (drivable_img == 2)] = 1
    road[drivable_img == 255] = 255
    return road


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default="C:/Users/Kanishk/Downloads/bdd100k/10k")
    ap.add_argument("--drivable", default="C:/Users/Kanishk/Downloads/bdd100k_drivable_maps")
    ap.add_argument("--labels", default="C:/Users/Kanishk/Downloads/bdd100k_labels/100k")
    ap.add_argument("--out", default="data_processed/bdd")
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args()

    # Potential label directories
    label_dirs = [
        args.labels,
        os.path.join(args.labels, "train"),
        os.path.join(args.labels, "val"),
        "C:/Users/Kanishk/Downloads/bdd100k_labels/100k/train",
        "C:/Users/Kanishk/Downloads/bdd100k_labels/100k/val",
    ]

    # Potential drivable directories
    drivable_dirs = [
        args.drivable,
        os.path.join(args.drivable, "labels", "train"),
        os.path.join(args.drivable, "labels", "val"),
    ]

    # Find candidate images that have detection labels
    print(f"Scanning images in {args.images}...")
    found_items = []
    for split_dir in ["train", "val", "test"]:
        p = os.path.join(args.images, split_dir)
        if not os.path.isdir(p):
            continue
        for f in os.listdir(p):
            if not f.endswith(".jpg"):
                continue
            stem = os.path.splitext(f)[0]
            lbl_file = find_file(label_dirs, f"{stem}.json")
            if lbl_file is not None:
                driv_file = find_file(drivable_dirs, f"{stem}_drivable_id.png")
                found_items.append({
                    "stem": stem,
                    "img_path": os.path.join(p, f),
                    "lbl_path": lbl_file,
                    "driv_path": driv_file,
                })

    print(f"Found {len(found_items)} images with detection labels.")
    if len(found_items) == 0:
        print("No matching images with labels found. Check paths.")
        return

    # Group by video ID (prefix before dash)
    video_groups = {}
    for item in found_items:
        vid = item["stem"].split("-")[0] if "-" in item["stem"] else item["stem"]
        video_groups.setdefault(vid, []).append(item)

    rng = random.Random(SEED)
    grouped_splits = split_groups(sorted(video_groups.keys()), rng, fracs=(0.70, 0.15, 0.15))
    chosen = {
        s: [item for vid in grouped_splits[s] for item in video_groups[vid]]
        for s in ("train", "val", "test")
    }

    make_dirs(args.out, chosen, ("images", "labels_det", "masks"))
    splits = {s: [] for s in chosen}

    for split, items in chosen.items():
        if args.limit:
            items = rng.sample(items, min(args.limit, len(items)))

        total_boxes = 0
        total_drivable = 0

        for i, item in enumerate(items):
            stem = item["stem"]
            img = cv2.imread(item["img_path"])
            if img is None:
                continue

            h, w = img.shape[:2]
            raw_classes, raw_boxes = load_bdd_sample(item["lbl_path"], img_w=w, img_h=h)

            road_mask = None
            if item["driv_path"] and os.path.exists(item["driv_path"]):
                d_img = cv2.imread(item["driv_path"], cv2.IMREAD_UNCHANGED)
                if d_img is not None:
                    road_mask = to_road_mask(d_img)
                    total_drivable += 1

            out = letterbox(img, boxes=raw_boxes, seg_mask=road_mask)
            clean_cls, clean_bx = clean_boxes(raw_classes, out["boxes"])

            save_sample(args.out, split, stem, out["image"], clean_cls, clean_bx)
            if out.get("seg_mask") is not None:
                cv2.imwrite(os.path.join(args.out, "masks", split, stem + ".png"), out["seg_mask"])

            splits[split].append(stem)
            total_boxes += len(clean_bx)

            if (i + 1) % 500 == 0 or (i + 1) == len(items):
                print(f"  {split}: {i + 1}/{len(items)} processed")

        print(f"{split}: {len(splits[split])} images, {total_boxes} boxes, {total_drivable} road masks")

    write_splits(args.out, splits)
    print("BDD100k conversion complete ->", args.out)


if __name__ == "__main__":
    main()
