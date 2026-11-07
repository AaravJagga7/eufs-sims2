"""Export the SegFormer road baseline to ONNX (input 1x3x384x640, Rule 5 size).

  python -m train.export_onnx_seg --checkpoint <run_dir>/best.pt
Writes best.onnx next to the checkpoint and checks it matches PyTorch.
"""
import argparse
import os

import numpy as np
import onnxruntime as ort
import torch

from datasets.contract import IMG_H, IMG_W
from models.wrappers import SegBaseline


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    args = p.parse_args()

    model = SegBaseline(args.checkpoint, device="cpu").model.eval()
    out_path = os.path.join(os.path.dirname(args.checkpoint), "best.onnx")
    dummy = torch.rand(1, 3, IMG_H, IMG_W)
    torch.onnx.export(model, (dummy,), out_path, input_names=["images"],
                      output_names=["logits"], opset_version=17, dynamo=False)

    with torch.no_grad():
        ref = model(dummy).numpy()
    got = ort.InferenceSession(out_path, providers=["CPUExecutionProvider"]).run(
        None, {"images": dummy.numpy()})[0]
    diff = float(np.abs(ref - got).max())
    print(f"Saved {out_path}  (max difference vs PyTorch: {diff:.2e})")
    if diff > 1e-3:
        raise RuntimeError("ONNX output does not match PyTorch")


if __name__ == "__main__":
    main()
