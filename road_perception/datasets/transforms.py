"""Image changes that keep image, boxes, road mask and distance map in step."""
import random
import cv2
import numpy as np

from datasets.contract import IMG_H, IMG_W

PAD_IMAGE = 114   # grey (same as YOLO); works for RGB or BGR
PAD_MASK = 255    # "ignore"
PAD_DEPTH = 0     # "no value"


def letterbox(image, boxes=None, seg_mask=None, depth=None, size=(IMG_H, IMG_W)):
    """Resize to `size` (height, width) by padding, never stretching.

    image:    H x W x 3 uint8
    boxes:    [N, 4] YOLO format (cx, cy, w, h), all 0-1 relative to the ORIGINAL image
    seg_mask: H x W uint8 (values 0 / 1 / 255); resized with nearest-neighbour
    depth:    H x W uint16 or float32 (metres); resized with nearest-neighbour.
              Metres do not change when an image is scaled, so values are kept as they are.

    Returns a dict with the same items moved the same way, plus
    "scale" and "pad" (left, top) so results can be mapped back to the original.
    """
    out_h, out_w = size
    h, w = image.shape[:2]
    scale = min(out_w / w, out_h / h)
    new_w, new_h = max(1, round(w * scale)), max(1, round(h * scale))
    pad_left, pad_top = (out_w - new_w) // 2, (out_h - new_h) // 2

    def place(arr, interpolation, pad_value):
        resized = cv2.resize(arr, (new_w, new_h), interpolation=interpolation)
        canvas = np.full((out_h, out_w) + arr.shape[2:], pad_value, dtype=arr.dtype)
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
        return canvas

    interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    out = {"image": place(image, interp, PAD_IMAGE), "scale": scale, "pad": (pad_left, pad_top)}

    if boxes is not None:
        b = np.asarray(boxes, dtype=np.float32).reshape(-1, 4).copy()
        b[:, 0] = (b[:, 0] * new_w + pad_left) / out_w
        b[:, 1] = (b[:, 1] * new_h + pad_top) / out_h
        b[:, 2] = b[:, 2] * new_w / out_w
        b[:, 3] = b[:, 3] * new_h / out_h
        out["boxes"] = b
    if seg_mask is not None:
        out["seg_mask"] = place(seg_mask, cv2.INTER_NEAREST, PAD_MASK)
    if depth is not None:
        out["depth"] = place(depth, cv2.INTER_NEAREST, PAD_DEPTH)
    return out


def augment_multitask(image, boxes=None, classes=None, seg_mask=None, depth=None,
                      p_flip=0.5, zoom_range=(0.8, 1.2), p_color=0.5, p_blur=0.2, rng=None):
    """Day 12 multitask augmentations:
    - Flip left-right
    - Zoom in/out 0.8-1.2x
    - Color jitter (brightness, contrast, saturation)
    - Blur

    Boxes, road mask and distance map stay strictly in sync. No upside-down flips, no rotations.
    """
    if rng is None:
        rng = random.Random()

    img = image.copy()
    h, w = img.shape[:2]

    b = None if boxes is None else np.asarray(boxes, dtype=np.float32).reshape(-1, 4).copy()
    cls = None if classes is None else np.asarray(classes, dtype=np.int64).copy()
    m = None if seg_mask is None else seg_mask.copy()
    d = None if depth is None else depth.copy()

    # 1. Horizontal Flip
    if rng.random() < p_flip:
        img = cv2.flip(img, 1)
        if b is not None and len(b) > 0:
            b[:, 0] = 1.0 - b[:, 0]
        if m is not None:
            m = cv2.flip(m, 1)
        if d is not None:
            d = cv2.flip(d, 1)

    # 2. Zoom In / Out (0.8x to 1.2x)
    scale = rng.uniform(zoom_range[0], zoom_range[1])
    if abs(scale - 1.0) > 0.02:
        new_w, new_h = max(1, round(w * scale)), max(1, round(h * scale))

        if scale > 1.0:
            # Zoom in: scale up and center crop
            crop_x = (new_w - w) // 2
            crop_y = (new_h - h) // 2
            img_res = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            img = img_res[crop_y : crop_y + h, crop_x : crop_x + w]

            if m is not None:
                m_res = cv2.resize(m, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                m = m_res[crop_y : crop_y + h, crop_x : crop_x + w]
            if d is not None:
                d_res = cv2.resize(d, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                d = d_res[crop_y : crop_y + h, crop_x : crop_x + w]

            if b is not None and len(b) > 0:
                # Convert normalized cx, cy, w, h to scaled coordinates
                cx = (b[:, 0] * new_w - crop_x) / w
                cy = (b[:, 1] * new_h - crop_y) / h
                bw = b[:, 2] * scale
                bh = b[:, 3] * scale

                x1 = np.clip(cx - bw / 2.0, 0.0, 1.0)
                y1 = np.clip(cy - bh / 2.0, 0.0, 1.0)
                x2 = np.clip(cx + bw / 2.0, 0.0, 1.0)
                y2 = np.clip(cy + bh / 2.0, 0.0, 1.0)

                valid_box = (x2 > x1 + 1e-4) & (y2 > y1 + 1e-4)
                if cls is not None:
                    cls = cls[valid_box]
                b = np.stack([(x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1], axis=1)[valid_box]

        else:
            # Zoom out: scale down and center pad
            pad_x = (w - new_w) // 2
            pad_y = (h - new_h) // 2
            img_res = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
            canvas = np.full((h, w, 3), PAD_IMAGE, dtype=img.dtype)
            canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = img_res
            img = canvas

            if m is not None:
                m_res = cv2.resize(m, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                canvas_m = np.full((h, w), PAD_MASK, dtype=m.dtype)
                canvas_m[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = m_res
                m = canvas_m
            if d is not None:
                d_res = cv2.resize(d, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                canvas_d = np.full((h, w), PAD_DEPTH, dtype=d.dtype)
                canvas_d[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = d_res
                d = canvas_d

            if b is not None and len(b) > 0:
                cx = (b[:, 0] * new_w + pad_x) / w
                cy = (b[:, 1] * new_h + pad_y) / h
                bw = b[:, 2] * scale
                bh = b[:, 3] * scale
                b = np.stack([cx, cy, bw, bh], axis=1)

    # 3. Color changes (image only)
    if rng.random() < p_color:
        # Random brightness & contrast
        alpha = rng.uniform(0.8, 1.2)   # contrast
        beta = rng.uniform(-20, 20)     # brightness
        img = np.clip(img.astype(np.float32) * alpha + beta, 0, 255).astype(np.uint8)

    # 4. Blur (image only)
    if rng.random() < p_blur:
        ksize = rng.choice([3, 5])
        img = cv2.GaussianBlur(img, (ksize, ksize), 0)

    return {
        "image": img,
        "boxes": b,
        "classes": cls,
        "seg_mask": m,
        "depth": d,
    }


class MultitaskAugmentor:
    """Callable transform object for MultiTaskDataset."""

    def __init__(self, p_flip=0.5, zoom_range=(0.8, 1.2), p_color=0.5, p_blur=0.2, seed=None):
        self.p_flip = p_flip
        self.zoom_range = zoom_range
        self.p_color = p_color
        self.p_blur = p_blur
        self.rng = random.Random(seed) if seed is not None else random.Random()

    def __call__(self, sample):
        return augment_multitask(
            sample["image"],
            boxes=sample.get("boxes"),
            classes=sample.get("classes"),
            seg_mask=sample.get("mask"),
            depth=sample.get("depth"),
            p_flip=self.p_flip,
            zoom_range=self.zoom_range,
            p_color=self.p_color,
            p_blur=self.p_blur,
            rng=self.rng,
        )
