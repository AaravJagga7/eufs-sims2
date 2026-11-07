"""Joint test (owned by Person A). Uses fake data now; on Integration Day 1
the fake dataset is replaced by Person B's MultiTaskDataset."""
import torch
from torch.utils.data import DataLoader, Dataset

from datasets.contract import IMG_H, IMG_W, empty_labels, check_sample, collate_fn
from models.wrappers import YoloBaseline


class FakeDataset(Dataset):
    """Samples that follow the contract, with random content."""

    def __len__(self):
        return 4

    def __getitem__(self, i):
        s = {"image": torch.rand(3, IMG_H, IMG_W), **empty_labels(),
             "has_det": True, "has_seg": True, "has_depth": True,
             "dataset": "fake", "stem": f"fake_{i}"}
        s["boxes"] = torch.tensor([[0.5, 0.5, 0.2, 0.1]])
        s["labels"] = torch.tensor([0])
        s["seg_mask"] = torch.randint(0, 2, (IMG_H, IMG_W))
        s["depth"] = torch.rand(IMG_H, IMG_W) * 80
        return s


def check_prediction(pred, batch_size):
    """Check a model's output follows Rule 5."""
    assert set(pred) == {"boxes", "seg", "depth"}
    assert len(pred["boxes"]) == batch_size
    for b in pred["boxes"]:
        assert b.ndim == 2 and b.shape[1] == 6
    if pred["seg"] is not None:
        assert pred["seg"].shape == (batch_size, IMG_H, IMG_W)
    if pred["depth"] is not None:
        assert pred["depth"].shape == (batch_size, IMG_H, IMG_W)


def test_data_follows_contract():
    ds = FakeDataset()
    for i in range(len(ds)):
        check_sample(ds[i])


def test_batch_shapes():
    batch = next(iter(DataLoader(FakeDataset(), batch_size=2, collate_fn=collate_fn)))
    assert batch["img"].shape == (2, 3, IMG_H, IMG_W)
    assert batch["det_targets"].shape[1] == 6


def test_yolo_baseline_output():
    batch = next(iter(DataLoader(FakeDataset(), batch_size=2, collate_fn=collate_fn)))
    model = YoloBaseline("yolo11n.pt", device="cpu")
    check_prediction(model.predict(batch["img"]), batch_size=2)
    

def test_seg_baseline_output():
    from models.wrappers import SegBaseline

    batch = next(iter(DataLoader(FakeDataset(), batch_size=2, collate_fn=collate_fn)))
    model = SegBaseline(device="cpu")   # untrained, no download needed
    pred = model.predict(batch["img"])
    check_prediction(pred, batch_size=2)
    assert pred["seg"].dtype == torch.int64


def test_multitask_output():
    from models.wrappers import MultiTaskWrapper

    batch = next(iter(DataLoader(FakeDataset(), batch_size=2, collate_fn=collate_fn)))
    model = MultiTaskWrapper(device="cpu")   # untrained, no download needed
    pred = model.predict(batch["img"])
    check_prediction(pred, batch_size=2)
    assert pred["seg"] is not None and pred["depth"] is not None
    assert pred["depth"].min() >= 0 and pred["depth"].max() <= 80


def test_multitask_loss_skips_missing_labels():
    from losses.multitask_loss import MultiTaskLoss
    from models.multitask import MultiTaskNet

    model = MultiTaskNet(pretrained=False).train()
    criterion = MultiTaskLoss(model)
    batch = next(iter(DataLoader(FakeDataset(), batch_size=2, collate_fn=collate_fn)))
    batch["has_seg"][:] = False          # pretend this batch has no road masks
    total, parts = criterion(model(batch["img"]), batch)
    total.backward()
    assert parts["seg"] is None and parts["det"] is not None and parts["depth"] is not None
    assert torch.isfinite(total)
    assert all(p.grad is None or p.grad.abs().sum() == 0 for p in model.seg_head.parameters())
    assert any(p.grad is not None and p.grad.abs().sum() > 0 for p in model.depth_head.parameters())
