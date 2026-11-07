import argparse
import os
import shutil
import torch  

from ultralytics import YOLO

from train.config import load_config


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--project", default=None, help="where to save, e.g. a Drive folder")
    p.add_argument("--epochs", type=int, default=None)
    p.add_argument("--batch", type=int, default=None)
    args = p.parse_args()

    cfg = load_config(args.config, project=args.project,
                      epochs=args.epochs, batch=args.batch)
    cfg["project"] = os.path.abspath(cfg["project"])
    run_dir = os.path.join(cfg["project"], cfg["run_name"])
    last = os.path.join(run_dir, "weights", "last.pt")

    if os.path.exists(last):
        ckpt = torch.load(last, map_location="cpu", weights_only=False)
        if ckpt.get("optimizer") is None:
            print(f"Run already finished: {run_dir}. Nothing to do.")
        else:
            print(f"Found unfinished run. Resuming from {last}")
            YOLO(last).train(resume=True)
    else:
        print(f"Starting new run: {cfg['run_name']}")
        YOLO(cfg["model"]).train(
            data=cfg["data"], imgsz=cfg["imgsz"], epochs=cfg["epochs"],
            batch=cfg["batch"], patience=cfg["patience"],
            save_period=cfg["save_period"], seed=cfg["seed"],
            project=cfg["project"], name=cfg["run_name"], exist_ok=True,
        )

    # Copy best.pt to the agreed place: checkpoints/<run_name>/best.pt (Rule 6)
    best = os.path.join(run_dir, "weights", "best.pt")
    if os.path.exists(best):
        shutil.copy(best, os.path.join(run_dir, "best.pt"))
        print(f"Saved {os.path.join(run_dir, 'best.pt')}")


if __name__ == "__main__":
    main()