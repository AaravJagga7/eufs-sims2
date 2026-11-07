"""Every model is wrapped so that predict(img) returns the same format (Rule 5):
  boxes: list of B tensors [K, 6] = x1, y1, x2, y2, score, class (pixels)
  seg:   int64 [B, 384, 640] (0 = not road, 1 = road), or None
  depth: float32 [B, 384, 640] in metres, or None
img is float32 [B, 3, 384, 640], values 0-1."""
import torch
from ultralytics import YOLO

from datasets.contract import IMG_H, IMG_W


class ModelWrapper:
    tasks = ()  # which jobs this model does: "det", "seg", "depth"

    def predict(self, img):
        raise NotImplementedError


class YoloBaseline(ModelWrapper):
    tasks = ("det",)

    def __init__(self, weights, conf=0.25, iou=0.7, device=None):
        self.model = YOLO(weights)
        self.conf, self.iou = conf, iou
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

    @torch.no_grad()
    def predict(self, img):
        results = self.model.predict(img, imgsz=(IMG_H, IMG_W), conf=self.conf,
                                     iou=self.iou, device=self.device, verbose=False)
        boxes = []
        for r in results:
            b = r.boxes
            if len(b):
                boxes.append(torch.cat([b.xyxy, b.conf[:, None], b.cls[:, None]], 1).cpu())
            else:
                boxes.append(torch.zeros((0, 6)))
        return {"boxes": boxes, "seg": None, "depth": None}


class SegBaseline(ModelWrapper):
    """SegFormer road baseline. checkpoint=None gives an untrained model (for tests)."""
    tasks = ("seg",)

    def __init__(self, checkpoint=None, device=None):
        from models.segformer_baseline import SegFormerRoad

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        backbone = "nvidia/mit-b0"
        state = None
        if checkpoint is not None:
            ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
            backbone = ckpt["config"]["model"]
            state = ckpt["model"]
        self.model = SegFormerRoad(backbone, pretrained=False)
        if state is not None:
            self.model.load_state_dict(state)
        self.model.to(self.device).eval()

    @torch.no_grad()
    def predict(self, img):
        logits = self.model(img.to(self.device))
        seg = logits.argmax(1).cpu()
        boxes = [torch.zeros((0, 6)) for _ in range(img.shape[0])]
        return {"boxes": boxes, "seg": seg, "depth": None}


class MultiTaskWrapper(ModelWrapper):
    """The combined model. checkpoint=None gives an untrained model (for tests)."""

    def __init__(self, checkpoint=None, device=None, conf=0.25, iou=0.7):
        from models.multitask import MultiTaskNet

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.conf, self.iou = conf, iou
        tasks, state = ("det", "seg", "depth"), None
        if checkpoint is not None:
            ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
            tasks, state = tuple(ckpt["config"]["tasks"]), ckpt["model"]
        self.model = MultiTaskNet(pretrained=False, tasks=tasks)
        if state is not None:
            self.model.load_state_dict(state)
        self.model.to(self.device).eval()
        self.tasks = tasks

    @torch.no_grad()
    def predict(self, img):
        from ultralytics.utils.nms import non_max_suppression

        out = self.model(img.to(self.device))
        dets = non_max_suppression(out["det"][0], conf_thres=self.conf, iou_thres=self.iou)
        boxes = [d[:, :6].float().cpu() for d in dets]
        seg = out["seg"].argmax(1).cpu() if out["seg"] is not None else None
        depth = out["depth"][:, 0].float().cpu() if out["depth"] is not None else None
        return {"boxes": boxes, "seg": seg, "depth": depth}
