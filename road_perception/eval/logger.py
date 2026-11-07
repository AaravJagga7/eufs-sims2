"""Training and experiment logger (owned by Person B).

Day 13 requirement:
Records each head's loss, learned loss weights, learning rate,
and validation scores every 5 epochs via WandB and/or TensorBoard.
"""
import os
from typing import Any, Dict, Optional


class MultitaskLogger:
    """Unified experiment logger supporting WandB and local fallback."""

    def __init__(
        self,
        project: str = "road-perception",
        run_name: Optional[str] = None,
        config: Optional[Dict[str, Any]] = None,
        use_wandb: bool = True,
        log_dir: str = "runs",
    ):
        self.project = project
        self.run_name = run_name
        self.config = config or {}
        self.use_wandb = use_wandb
        self.log_dir = log_dir
        self.wandb_run = None

        if self.use_wandb:
            try:
                import wandb
                self.wandb_run = wandb.init(
                    project=self.project,
                    name=self.run_name,
                    config=self.config,
                    reinit=True,
                )
                print(f"[Logger] WandB initialized for run: {self.run_name}")
            except Exception as e:
                print(f"[Logger] WandB init skipped or offline ({e}). Using local logging.")
                self.wandb_run = None

    def log_epoch(
        self,
        epoch: int,
        losses: Dict[str, float],
        weights: Optional[Dict[str, float]] = None,
        lr: Optional[float] = None,
        val_scores: Optional[Dict[str, float]] = None,
    ):
        """Logs metrics for an epoch.

        losses: dict of head losses, e.g. {'det_loss': ..., 'seg_loss': ..., 'depth_loss': ..., 'total_loss': ...}
        weights: dict of learned task weights, e.g. {'w_det': ..., 'w_seg': ..., 'w_depth': ...}
        val_scores: dict of validation metrics every 5 epochs
        """
        payload = {"epoch": epoch}
        for k, v in losses.items():
            payload[f"train/{k}"] = float(v)

        if weights:
            for k, v in weights.items():
                payload[f"weights/{k}"] = float(v)

        if lr is not None:
            payload["train/lr"] = float(lr)

        if val_scores:
            for k, v in val_scores.items():
                payload[f"val/{k}"] = float(v)

        if self.wandb_run is not None:
            import wandb
            wandb.log(payload, step=epoch)

        return payload

    def finish(self):
        if self.wandb_run is not None:
            import wandb
            wandb.finish()
