"""Inference speed and parameter benchmarking (owned by Person B).

Measures:
- fps:        frames per second (1000 / latency_ms)
- latency_ms: milliseconds per frame
- params_M:   parameter count in millions

Benchmark procedure (Plan Day 9):
- 20 warm-up runs
- 200 timed runs with dummy input (1, 3, 384, 640)

Run standalone:
  python -m eval.benchmark_fps
"""
import argparse
import time
import numpy as np
import torch

from datasets.contract import IMG_H, IMG_W


def count_parameters(model):
    """Counts parameters in millions."""
    if hasattr(model, "parameters"):
        total = sum(p.numel() for p in model.parameters())
        return float(total / 1e6)
    elif hasattr(model, "model") and hasattr(model.model, "parameters"):
        total = sum(p.numel() for p in model.model.parameters())
        return float(total / 1e6)
    return 0.0


def benchmark_speed(model, n_warmup=20, n_runs=200, device=None):
    """Measures latency and FPS of model.predict()."""
    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"

    dummy_input = torch.rand(1, 3, IMG_H, IMG_W)

    # Warmup
    for _ in range(n_warmup):
        _ = model.predict(dummy_input)

    if device == "cuda":
        torch.cuda.synchronize()

    start = time.perf_counter()
    for _ in range(n_runs):
        _ = model.predict(dummy_input)
    if device == "cuda":
        torch.cuda.synchronize()
    total_time = time.perf_counter() - start

    avg_latency_ms = (total_time / n_runs) * 1000.0
    fps = 1000.0 / max(1e-6, avg_latency_ms)
    params_m = count_parameters(model)

    return {
        "fps": float(fps),
        "latency_ms": float(avg_latency_ms),
        "params_M": float(params_m),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--runs", type=int, default=20)
    args = parser.parse_args()

    class DummyPredictor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.conv = torch.nn.Conv2d(3, 16, 3, padding=1)

        def predict(self, imgs):
            with torch.no_grad():
                return {"boxes": None, "seg": None, "depth": None}

    model = DummyPredictor()
    res = benchmark_speed(model, n_warmup=args.warmup, n_runs=args.runs)
    print("Benchmark results:")
    for k, v in res.items():
        print(f"  {k}: {v:.2f}")
