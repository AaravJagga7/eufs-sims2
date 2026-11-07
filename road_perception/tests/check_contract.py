import torch
from datasets.contract import IMG_H, IMG_W, empty_labels, check_sample, collate_fn

# A fake sample with two boxes
s = {"image": torch.rand(3, IMG_H, IMG_W), **empty_labels(),
     "has_det": True, "has_seg": False, "has_depth": False,
     "dataset": "fake", "stem": "x"}
s["boxes"] = torch.tensor([[0.5, 0.5, 0.2, 0.1], [0.3, 0.7, 0.1, 0.1]])
s["labels"] = torch.tensor([0, 2])

check_sample(s)
b = collate_fn([s, s])
for k, v in b.items():
    print(k, getattr(v, "shape", v))
print("Contract OK")