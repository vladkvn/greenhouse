"""Person detector backed by Ultralytics YOLO running on the Jetson GPU.

Prefers a prebuilt TensorRT engine (.engine, FP16) for speed; falls back to the
.pt weights (still GPU, slower) when the engine is absent.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass

import numpy as np

from .config import FollowMeConfig

log = logging.getLogger(__name__)


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


class Detector:
    def __init__(self, cfg: FollowMeConfig):
        self._cfg = cfg
        weights = cfg.engine_path
        if not os.path.exists(weights):
            log.warning(
                "TensorRT engine '%s' not found -- falling back to '%s' (slower). "
                "Build the engine with scripts/export_yolo_tensorrt.py for full speed.",
                cfg.engine_path,
                cfg.fallback_weights,
            )
            weights = cfg.fallback_weights
        log.info("Loading detector weights: %s", weights)
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "ultralytics not installed. Run: pip install ultralytics"
            ) from exc
        self._model = YOLO(weights, task="detect")

    def detect(self, frame: np.ndarray) -> list[Detection]:
        cfg = self._cfg
        results = self._model(
            frame,
            conf=cfg.conf_thr,
            classes=[cfg.person_class_id],
            imgsz=cfg.infer_imgsz,
            device=cfg.device,
            verbose=False,
        )
        out: list[Detection] = []
        if not results:
            return out
        boxes = results[0].boxes
        if boxes is None or boxes.xyxy is None:
            return out
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        for (x1, y1, x2, y2), c in zip(xyxy, confs):
            out.append(Detection(float(x1), float(y1), float(x2), float(y2), float(c)))
        return out
