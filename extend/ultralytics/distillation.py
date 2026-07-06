"""
YOLO11s Detection Knowledge Distillation.

Extends a pretrained YOLO11s model with new classes using limited data
while preserving original detection capability via knowledge distillation.

Usage:
    from distillation import DistillationTrainer

    trainer = DistillationTrainer(
        teacher_weights="yolo11s.pt",
        old_nc=80,
        data="new_data.yaml",
        epochs=50,
        batch=16,
        imgsz=640,
        lr0=0.002,
    )
    trainer.train()
"""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from ultralytics.data import build_dataloader, build_yolo_dataset
from ultralytics.engine.trainer import BaseTrainer
from ultralytics.models import yolo
from ultralytics.models.yolo.detect import DetectionTrainer, DetectionValidator
from ultralytics.nn.modules.head import Detect
from ultralytics.nn.tasks import DetectionModel, load_checkpoint
from ultralytics.utils import DEFAULT_CFG, LOGGER, RANK, callbacks
from ultralytics.utils.autobatch import check_train_batch_size
from ultralytics.utils.checks import check_amp, check_file, check_imgsz
from ultralytics.utils.dist import ddp_cleanup, generate_ddp_command
from ultralytics.utils.loss import v8DetectionLoss
from ultralytics.utils.plotting import plot_images, plot_labels, plot_results
from ultralytics.utils.torch_utils import (
    TORCH_2_4,
    EarlyStopping,
    ModelEMA,
    autocast,
    init_seeds,
    intersect_dicts,
    one_cycle,
    select_device,
    strip_optimizer,
    torch_distributed_zero_first,
    unwrap_model,
)


# ---------------------------------------------------------------------------
#  Knowledge Distillation Model
# ---------------------------------------------------------------------------

class DistillationModel(DetectionModel):
    """
    DetectionModel extended with knowledge distillation for new class expansion.

    Wraps a frozen teacher model alongside a trainable student model.
    The student inherits backbone+neck weights from the teacher, with an
    expanded detection head to accommodate new classes.

    During training, the forward pass computes:
        - Standard YOLO detection loss (student vs ground truth)
        - Logit distillation loss (student old-class logits vs teacher logits)
        - Feature distillation loss (MSE between intermediate FPN features)

    Attributes:
        _old_nc (int): Number of classes the teacher was trained on.
        _teacher_weights (str): Path to teacher checkpoint.
        _teacher (DetectionModel): Frozen teacher model.
        _temperature (float): Temperature for softening KD logits.
        _kd_weight (float): Weight for logit distillation loss.
        _feat_weight (float): Weight for feature distillation loss.
        _feature_layers (tuple[int]): Indices of FPN layers to match features on.

    Examples:
        Create a distillation model for extending COCO (80 cls) to 85 cls.
        >>> model = DistillationModel(
        ...     cfg="yolo11.yaml",
        ...     teacher_weights="yolo11s.pt",
        ...     old_nc=80,
        ...     nc=85,
        ... )
    """

    def __init__(
        self,
        cfg: str | dict = "yolo11.yaml",
        teacher_weights: str = "yolo11s.pt",
        old_nc: int = 80,
        nc: int | None = None,
        ch: int = 3,
        verbose: bool = True,
        temperature: float = 3.0,
        kd_weight: float = 5.0,
        feat_weight: float = 1.0,
        feature_layers: tuple[int, ...] = (16, 19, 22),
    ) -> None:
        """
        Initialize the DistillationModel.

        Args:
            cfg: Model config file path or dict (e.g. 'yolo11.yaml').
            teacher_weights: Path to pretrained teacher checkpoint.
            old_nc: Number of classes in the teacher model.
            nc: Total number of classes for student (old_nc + new_nc).
            ch: Input channels.
            verbose: Print model info.
            temperature: KD temperature for softening logits.
            kd_weight: Weight multiplier for logit distillation loss.
            feat_weight: Weight multiplier for feature distillation loss.
            feature_layers: Indices of FPN layers whose features are matched.
        """
        # Store distillation hyperparameters before parent init
        self._old_nc = old_nc
        self._teacher_weights = teacher_weights
        self._teacher: DetectionModel | None = None
        self._temperature = temperature
        self._kd_weight = kd_weight
        self._feat_weight = feat_weight
        self._feature_layers = feature_layers

        # Parent init creates the student model architecture
        super().__init__(cfg=cfg, ch=ch, nc=nc, verbose=verbose)

        # Load teacher and copy weights into student
        self._init_teacher()

    # ------------------------------------------------------------------
    #  Properties (for external access, e.g. from trainer)
    # ------------------------------------------------------------------

    @property
    def old_nc(self) -> int:
        """Return the number of old (teacher) classes."""
        return self._old_nc

    @property
    def kd_weight(self) -> float:
        """Return the logit distillation loss weight."""
        return self._kd_weight

    @property
    def feat_weight(self) -> float:
        """Return the feature distillation loss weight."""
        return self._feat_weight

    # ------------------------------------------------------------------
    #  Teacher initialisation & weight transfer
    # ------------------------------------------------------------------

    def _init_teacher(self) -> None:
        """Load the frozen teacher model and copy its weights into the student."""
        from ultralytics import YOLO

        # --- load teacher ----------------------------------------------------
        LOGGER.info(f"Loading teacher model from {self._teacher_weights} ...")
        teacher = YOLO(self._teacher_weights).model
        teacher.eval()
        for p in teacher.parameters():
            p.requires_grad = False
        self._teacher = teacher

        # --- copy backbone + neck + cv2 + dfl (exact shape matches) ----------
        teacher_sd = teacher.state_dict()
        student_sd = self.state_dict()

        detect_idx = self._find_detect_idx()

        for key in student_sd:
            if key in teacher_sd and student_sd[key].shape == teacher_sd[key].shape:
                # Exact match – direct copy
                student_sd[key] = teacher_sd[key].clone()
            elif key in teacher_sd and f"model.{detect_idx}.cv3." in key and ".2." in key:
                # cv3.i.2 is the final Conv2d whose output channels depend on nc.
                # Copy old-class channels from teacher.
                if "weight" in key:
                    student_sd[key][: self._old_nc] = teacher_sd[key].clone()
                elif "bias" in key:
                    student_sd[key][: self._old_nc] = teacher_sd[key].clone()

        self.load_state_dict(student_sd, strict=True)
        n_copied = sum(1 for k in student_sd if k in teacher_sd)
        LOGGER.info(f"Transferred {n_copied} / {len(student_sd)} parameter tensors from teacher to student.")

    def _find_detect_idx(self) -> int:
        """Return the index of the Detect head inside self.model (nn.Sequential)."""
        for i, m in enumerate(self.model):
            if isinstance(m, Detect):
                return i
        # Fallback: last layer
        return len(self.model) - 1

    # ------------------------------------------------------------------
    #  Forward pass  (dispatches to _distill_forward for training)
    # ------------------------------------------------------------------

    def forward(self, x: torch.Tensor | dict, *args: Any, **kwargs: Any):
        """
        Forward pass.

        If x is a dict (training batch), runs the distillation pipeline.
        Otherwise runs normal inference via self.predict().
        """
        if isinstance(x, dict):
            return self._distill_forward(x)
        return self.predict(x, *args, **kwargs)

    def loss(self, batch: dict, preds: list[torch.Tensor] | None = None):
        """
        Compute loss, padding to 5 elements for validator compatibility.

        During validation, the standard DetectionModel.loss returns only 3
        elements (box, cls, dfl).  We pad with zeros for kd_loss and feat_loss
        so the validator's self.loss (initialized to 5 elements) matches.
        """
        total_loss, loss_items = DetectionModel.loss(self, batch, preds)
        if loss_items.numel() == 3:
            loss_items = torch.cat([loss_items, torch.zeros(2, device=loss_items.device)])
        return total_loss, loss_items

    # ------------------------------------------------------------------
    #  Distillation training forward
    # ------------------------------------------------------------------

    def _distill_forward(
        self, batch: dict[str, torch.Tensor]
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Run one distillation training step.

        Returns:
            (total_loss, loss_items): Total loss (scalar) and detached
            per-component loss tensor for logging.
        """
        img = batch["img"]

        # -- register feature hooks ------------------------------------------
        t_feats, t_handles = self._register_hooks(self._teacher)
        s_feats, s_handles = self._register_hooks(self)

        try:
            # -- teacher forward (no grad) -----------------------------------
            with torch.no_grad():
                teacher_preds = self._teacher.predict(img)

            # -- student forward ---------------------------------------------
            student_preds = self.predict(img)

            # -- detection loss (standard YOLO loss) -------------------------
            det_loss, det_loss_items = super().loss(batch, student_preds)

            # -- distillation losses -----------------------------------------
            kd_loss = self._compute_kd_loss(teacher_preds, student_preds)
            feat_loss = self._compute_feat_loss(t_feats, s_feats)

            # -- total loss --------------------------------------------------
            total_loss = det_loss + self._kd_weight * kd_loss + self._feat_weight * feat_loss

            # Combine for logging
            loss_items = torch.cat(
                [
                    det_loss_items,
                    kd_loss.detach().unsqueeze(0),
                    feat_loss.detach().unsqueeze(0),
                ]
            )
            return total_loss, loss_items

        finally:
            for h in t_handles + s_handles:
                h.remove()

    # ------------------------------------------------------------------
    #  Hook helpers
    # ------------------------------------------------------------------

    def _register_hooks(self, model: DetectionModel) -> tuple[dict[int, torch.Tensor], list]:
        """Register forward hooks on the feature layers of a model.

        Returns (features_dict, handles_list)."""
        features: dict[int, torch.Tensor] = {}
        handles: list = []

        def _make_hook(idx: int):
            def _hook(_module, _input, output):
                features[idx] = output

            return _hook

        for idx in self._feature_layers:
            if idx < len(model.model):
                h = model.model[idx].register_forward_hook(_make_hook(idx))
                handles.append(h)

        return features, handles

    # ------------------------------------------------------------------
    #  Loss computations
    # ------------------------------------------------------------------

    def _compute_kd_loss(
        self,
        teacher_preds: list[torch.Tensor],
        student_preds: list[torch.Tensor],
    ) -> torch.Tensor:
        """
        KL-divergence loss between teacher and student classification logits
        on old-class channels only, at each detection scale.

        Formula:  L_kd = T^2 * KL( softmax(s_logits_old / T) || softmax(t_logits / T) )
        """
        T = self._temperature
        reg_max = self.model[-1].reg_max  # 16
        device = next(self.parameters()).device
        total = torch.tensor(0.0, device=device)
        count = 0

        for t_out, s_out in zip(teacher_preds, student_preds):
            # teacher: [B, reg_max*4 + old_nc, H, W]
            # student: [B, reg_max*4 + old_nc + new_nc, H, W]
            t_cls = t_out[:, reg_max * 4 :, :, :]  # old_nc channels
            s_cls = s_out[:, reg_max * 4 :, :, :]  # old_nc + new_nc channels
            s_cls_old = s_cls[:, : self._old_nc, :, :]  # first old_nc channels

            B, C, H, W = t_cls.shape
            # Flatten spatial dimensions: (B, C, H, W) -> (B*H*W, C)
            t_flat = t_cls.permute(0, 2, 3, 1).reshape(-1, C) / T
            s_flat = s_cls_old.permute(0, 2, 3, 1).reshape(-1, C) / T

            total += (
                F.kl_div(
                    F.log_softmax(s_flat, dim=1),
                    F.softmax(t_flat, dim=1),
                    reduction="batchmean",
                )
                * T
                * T
            )
            count += 1

        return total / max(count, 1)

    def _compute_feat_loss(
        self,
        teacher_feats: dict[int, torch.Tensor],
        student_feats: dict[int, torch.Tensor],
    ) -> torch.Tensor:
        """MSE loss between teacher and student FPN feature maps at P3/P4/P5."""
        device = next(self.parameters()).device
        total = torch.tensor(0.0, device=device)
        count = 0

        for idx in self._feature_layers:
            t = teacher_feats.get(idx)
            s = student_feats.get(idx)
            if t is not None and s is not None:
                total += F.mse_loss(s, t)
                count += 1

        return total / max(count, 1)

    # ------------------------------------------------------------------
    #  Device transfer  (ensure teacher moves with student)
    # ------------------------------------------------------------------

    def to(self, *args: Any, **kwargs: Any):
        """Move student and teacher to the specified device/dtype."""
        super().to(*args, **kwargs)
        if self._teacher is not None:
            self._teacher.to(*args, **kwargs)
        return self

    def _apply(self, fn):
        """Apply a function to all tensors in student *and* teacher."""
        super()._apply(fn)
        if self._teacher is not None:
            self._teacher._apply(fn)
        return self


# ---------------------------------------------------------------------------
#  Knowledge Distillation Trainer
# ---------------------------------------------------------------------------

class DistillationTrainer(DetectionTrainer):
    """
    Trainer for YOLO detection knowledge distillation.

    Creates a DistillationModel (teacher + student) and manages the
    training loop with combined detection + distillation losses.

    Additional keyword arguments (passed through ``overrides``):
        teacher_weights (str): Path to teacher checkpoint (default: "yolo11s.pt").
        old_nc (int): Number of classes in the teacher model.
        temperature (float): KD temperature (default: 3.0).
        kd_weight (float): Logit KD loss weight (default: 5.0).
        feat_weight (float): Feature KD loss weight (default: 1.0).

    Examples:
        >>> from distillation import DistillationTrainer
        >>> trainer = DistillationTrainer(
        ...     teacher_weights="yolo11s.pt",
        ...     old_nc=80,
        ...     data="new_data.yaml",
        ...     epochs=50,
        ...     batch=16,
        ...     imgsz=640,
        ...     lr0=0.002,
        ... )
        >>> trainer.train()
    """

    def __init__(
        self,
        teacher_weights: str = "yolo11s.pt",
        old_nc: int = 80,
        temperature: float = 3.0,
        kd_weight: float = 5.0,
        feat_weight: float = 1.0,
        cfg: str = DEFAULT_CFG,
        overrides: dict | None = None,
        _callbacks: list | None = None,
    ) -> None:
        """
        Initialize the DistillationTrainer.

        Args:
            teacher_weights: Path to teacher model checkpoint.
            old_nc: Number of classes the teacher was trained on.
            temperature: KD temperature parameter.
            kd_weight: Weight for logit distillation loss.
            feat_weight: Weight for feature distillation loss.
            cfg: Base config file path.
            overrides: Additional overrides for training configuration.
            _callbacks: Callback functions.
        """
        # Store distillation args before parent __init__
        self._distill_teacher_weights = teacher_weights
        self._distill_old_nc = old_nc
        self._distill_temperature = temperature
        self._distill_kd_weight = kd_weight
        self._distill_feat_weight = feat_weight

        if overrides is None:
            overrides = {}
        overrides["task"] = "detect"
        overrides.setdefault("model", teacher_weights)  # prevent BaseTrainer.__init__ crash with model=None

        super().__init__(cfg, overrides, _callbacks)

    # ------------------------------------------------------------------
    #  Model construction
    # ------------------------------------------------------------------

    def get_model(
        self, cfg: str | None = None, weights: str | None = None, verbose: bool = True
    ) -> DistillationModel:
        """
        Build a DistillationModel with the teacher checkpoint.

        The student architecture follows the same cfg as the teacher
        but with an expanded detection head (nc = old_nc + new_nc).
        """
        model = DistillationModel(
            cfg=cfg or "yolo11.yaml",
            teacher_weights=self._distill_teacher_weights,
            old_nc=self._distill_old_nc,
            nc=None,  # set later via set_model_attributes
            verbose=verbose and RANK == -1,
            temperature=self._distill_temperature,
            kd_weight=self._distill_kd_weight,
            feat_weight=self._distill_feat_weight,
        )
        return model

    def set_model_attributes(self) -> None:
        """
        Set model attributes after the dataset is loaded.

        Derives new_nc from the dataset's total class count and
        completes the student model construction with the correct nc.
        """
        total_nc = self.data["nc"]
        old_nc = self._distill_old_nc
        new_nc = total_nc - old_nc

        if new_nc < 0:
            raise ValueError(
                f"Dataset has {total_nc} classes, but teacher has {old_nc}. "
                f"Dataset class count must be >= teacher class count ({old_nc})."
            )

        LOGGER.info(
            f"Distillation config: old_nc={old_nc}, new_nc={new_nc}, total_nc={total_nc}"
        )

        # Rebuild the student with correct nc
        student_cfg = copy.deepcopy(self.model.yaml)
        student_cfg["nc"] = total_nc

        new_student = DetectionModel(student_cfg, ch=3, nc=total_nc, verbose=False)
        new_student = new_student.to(self.device)

        # Copy weights: exact matches (backbone/neck/cv2/dfl) + cv3 old-class channels
        teacher = self.model._teacher
        teacher_sd = teacher.state_dict()
        student_sd = new_student.state_dict()

        detect_idx = self.model._find_detect_idx()
        for key in student_sd:
            if key in teacher_sd and student_sd[key].shape == teacher_sd[key].shape:
                student_sd[key] = teacher_sd[key].clone().to(self.device)
            elif key in teacher_sd and f"model.{detect_idx}.cv3." in key and ".2." in key:
                if "weight" in key:
                    student_sd[key][:old_nc] = teacher_sd[key].clone().to(self.device)
                elif "bias" in key:
                    student_sd[key][:old_nc] = teacher_sd[key].clone().to(self.device)

        new_student.load_state_dict(student_sd, strict=True)

        # Replace internal student components in the DistillationModel wrapper
        self.model.model = new_student.model
        self.model.save = new_student.save
        self.model.yaml = new_student.yaml
        self.model.names = self.data["names"]
        self.model.nc = total_nc
        self.model.args = self.args
        self.model._old_nc = old_nc
        self.model.stride = new_student.stride

        # Persist distill config for checkpointing
        self.model._distill_config = {
            "teacher_weights": self._distill_teacher_weights,
            "old_nc": old_nc,
            "temperature": self._distill_temperature,
            "kd_weight": self._distill_kd_weight,
            "feat_weight": self._distill_feat_weight,
        }

    # ------------------------------------------------------------------
    #  Loss naming
    # ------------------------------------------------------------------

    def get_validator(self) -> DetectionValidator:
        """Return a DetectionValidator with extended loss names."""
        self.loss_names = ("box_loss", "cls_loss", "dfl_loss", "kd_loss", "feat_loss")
        return DetectionValidator(
            self.test_loader,
            save_dir=self.save_dir,
            args=copy.copy(self.args),
            _callbacks=self.callbacks,
        )

    def label_loss_items(
        self, loss_items: torch.Tensor | None = None, prefix: str = "train"
    ) -> list[str] | dict[str, float]:
        """Return labeled loss items dictionary or list of keys."""
        keys = [f"{prefix}/{x}" for x in self.loss_names]
        if loss_items is not None:
            loss_items = [round(float(x), 5) for x in loss_items]
            return dict(zip(keys, loss_items))
        return keys

    # ------------------------------------------------------------------
    #  Validation
    # ------------------------------------------------------------------

    def validate(self) -> tuple:
        """Run validation on the student model (inference-only, no teacher).

        Follows the standard BaseTrainer.validate() pattern:
        passes self to validator which extracts model + ema internally."""
        metrics = self.validator(self)
        if metrics is None:
            return None, None
        fitness = metrics.pop("fitness", -self.loss.detach().cpu().numpy())
        if not self.best_fitness or self.best_fitness < fitness:
            self.best_fitness = fitness
        return metrics, fitness

    # ------------------------------------------------------------------
    #  Checkpoint save / resume
    # ------------------------------------------------------------------

    def save_model(self) -> None:
        """Save checkpoint with a clean DetectionModel so YOLO(\"best.pt\") works.

        DistillationModel is a custom class not available to pickle in other
        scripts.  We strip it to a plain DetectionModel containing only the
        student weights.
        """
        import io
        from copy import deepcopy
        from datetime import datetime

        from ultralytics import __version__
        from ultralytics.utils.torch_utils import convert_optimizer_state_dict_to_fp16

        yaml_config = deepcopy(self.model.yaml)
        yaml_config["nc"] = self.model.nc

        # --- build a clean DetectionModel from the EMA student weights ---------
        ema_model: DistillationModel = unwrap_model(self.ema.ema)
        clean_model = DetectionModel(yaml_config, ch=3, nc=self.model.nc, verbose=False)
        # Filter out teacher-prefixed keys ("_teacher.*")
        ema_sd = ema_model.state_dict()
        clean_sd = {k: v for k, v in ema_sd.items() if not k.startswith("_teacher")}
        clean_model.load_state_dict(intersect_dicts(clean_sd, clean_model.state_dict()), strict=False)

        ckpt = {
            "epoch": self.epoch,
            "best_fitness": self.best_fitness,
            "model": None,  # final checkpoints derive from EMA
            "ema": deepcopy(clean_model).half(),
            "updates": self.ema.updates,
            "optimizer": convert_optimizer_state_dict_to_fp16(deepcopy(self.optimizer.state_dict())),
            "scaler": self.scaler.state_dict(),
            "train_args": vars(self.args),
            "train_metrics": {**self.metrics, **{"fitness": self.fitness}},
            "train_results": self.read_results_csv(),
            "date": datetime.now().isoformat(),
            "version": __version__,
            "license": "AGPL-3.0 (https://ultralytics.com/license)",
            "docs": "https://docs.ultralytics.com",
            "yaml": yaml_config,
            "distill_config": getattr(self.model, "_distill_config", {}),
        }

        buffer = io.BytesIO()
        torch.save(ckpt, buffer)
        serialized = buffer.getvalue()

        self.last.write_bytes(serialized)
        if self.best_fitness == self.fitness:
            self.best.write_bytes(serialized)

    def resume_training(self, ckpt: dict | None) -> None:
        """Resume from checkpoint, restoring student weights."""
        if ckpt is None or not self.args.resume:
            return
        if ckpt.get("ema") is not None:
            # Load saved EMA (DetectionModel) state_dict into student (DistillationModel)
            ema_sd = ckpt["ema"].float().state_dict()
            unwrap_model(self.model).load_state_dict(ema_sd, strict=False)
            self.ema.ema.load_state_dict(ema_sd, strict=False)
            self.ema.updates = ckpt.get("updates", self.ema.updates)
        if ckpt.get("optimizer") is not None:
            self.optimizer.load_state_dict(ckpt["optimizer"])
        self.best_fitness = ckpt.get("best_fitness", 0.0)
        self.start_epoch = ckpt["epoch"] + 1
        LOGGER.info(f"Resumed training from epoch {self.start_epoch}")


# ---------------------------------------------------------------------------
#  Utility: load distilled student model for inference
# ---------------------------------------------------------------------------


def load_distilled_model(ckpt_path: str, device: str | torch.device = "cpu") -> DetectionModel:
    """
    Load a distilled student model from a checkpoint for inference.

    Args:
        ckpt_path: Path to the checkpoint file (e.g. 'best.pt').
        device: Device to load the model on.

    Returns:
        DetectionModel ready for inference.

    Examples:
        >>> model = load_distilled_model("runs/detect/distill/weights/best.pt", device="cuda:0")
        >>> results = model.predict("image.jpg")
    """
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)

    # Use the saved yaml config if available, otherwise fall back to yolo11.yaml
    yaml_config = ckpt.get("yaml")
    if yaml_config is None:
        # Backward compatibility: try to infer from checkpoint
        yaml_config = "yolo11.yaml"
        LOGGER.warning(
            f"No yaml config in checkpoint, defaulting to {yaml_config}. "
            "This may fail if class count differs from the yaml default."
        )

    nc = yaml_config.get("nc", 80) if isinstance(yaml_config, dict) else 80
    model = DetectionModel(yaml_config, ch=3, nc=nc, verbose=False)

    csd = ckpt["model"].float().state_dict() if isinstance(ckpt["model"], nn.Module) else ckpt["model"]
    model.load_state_dict(csd, strict=True)

    if "names" not in yaml_config and "train_args" in ckpt:
        # Try to restore class names from saved config
        ...

    model.to(device)
    model.eval()
    LOGGER.info(f"Loaded distilled model: {nc} classes from {ckpt_path}")
    return model


# ---------------------------------------------------------------------------
#  Convenience training function
# ---------------------------------------------------------------------------

def train_distillation(
    teacher_weights: str = "yolo11s.pt",
    old_nc: int = 80,
    data: str = "coco8.yaml",
    epochs: int = 50,
    batch: int = 16,
    imgsz: int = 640,
    lr0: float = 0.002,
    warmup_epochs: int = 3,
    temperature: float = 3.0,
    kd_weight: float = 5.0,
    feat_weight: float = 1.0,
    name: str = "distill",
    device: int | str = 0,
    **kwargs,
) -> dict:
    """
    Convenience function to train a YOLO11s distillation model.

    Args:
        teacher_weights: Path to teacher model checkpoint.
        old_nc: Number of old classes in the teacher model.
        data: Path to dataset YAML config.
        epochs: Number of training epochs.
        batch: Batch size.
        imgsz: Image size.
        lr0: Initial learning rate.
        warmup_epochs: Number of warmup epochs.
        temperature: KD temperature.
        kd_weight: Logit distillation loss weight.
        feat_weight: Feature distillation loss weight.
        name: Experiment name for save directory.
        device: GPU device (e.g. 0, '0,1').
        **kwargs: Additional trainer arguments.

    Returns:
        Training metrics dictionary.
    """
    overrides = {
        "model": teacher_weights,  # prevent BaseTrainer.__init__ crash with model=None
        "data": data,
        "epochs": epochs,
        "batch": batch,
        "imgsz": imgsz,
        "lr0": lr0,
        "warmup_epochs": warmup_epochs,
        "name": name,
        "device": device,
        "plots": True,
        "save": True,
        **kwargs,
    }

    trainer = DistillationTrainer(
        teacher_weights=teacher_weights,
        old_nc=old_nc,
        temperature=temperature,
        kd_weight=kd_weight,
        feat_weight=feat_weight,
        overrides=overrides,
    )
    trainer.train()
    return trainer.metrics


# ---------------------------------------------------------------------------
#  CLI entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="YOLO11s Detection Knowledge Distillation"
    )
    parser.add_argument(
        "--teacher", type=str, default="yolo11s.pt", help="Teacher checkpoint path"
    )
    parser.add_argument(
        "--old-nc", type=int, default=80, help="Number of teacher classes"
    )
    parser.add_argument(
        "--data", type=str, required=True, help="Dataset YAML path"
    )
    parser.add_argument(
        "--epochs", type=int, default=50, help="Training epochs"
    )
    parser.add_argument(
        "--batch", type=int, default=4, help="Batch size"
    )
    parser.add_argument(
        "--imgsz", type=int, default=480, help="Image size"
    )
    parser.add_argument(
        "--lr0", type=float, default=0.002, help="Initial learning rate"
    )
    parser.add_argument(
        "--warmup", type=int, default=3, help="Warmup epochs"
    )
    parser.add_argument(
        "--temperature", type=float, default=3.0, help="KD temperature"
    )
    parser.add_argument(
        "--kd-weight", type=float, default=5.0, help="KD loss weight"
    )
    parser.add_argument(
        "--feat-weight", type=float, default=1.0, help="Feature loss weight"
    )
    parser.add_argument(
        "--name", type=str, default="distill", help="Experiment name"
    )
    parser.add_argument(
        "--device", type=str, default="0", help="GPU device"
    )
    parser.add_argument(
        "--freeze", type=int, default=10, help="Freeze first N layers"
    )
    parser.add_argument(
        "--patience", type=int, default=10, help="Early stopping patience"
    )

    args = parser.parse_args()

    metrics = train_distillation(
        teacher_weights=args.teacher,
        old_nc=args.old_nc,
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        lr0=args.lr0,
        warmup_epochs=args.warmup,
        temperature=args.temperature,
        kd_weight=args.kd_weight,
        feat_weight=args.feat_weight,
        name=args.name,
        device=args.device,
        freeze=args.freeze,
        patience=args.patience,
    )

    print(f"\nTraining complete. Final metrics: {metrics}")
