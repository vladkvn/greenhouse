"""JetsonTargetDetector — детектор цели (человека) для режима FOLLOWING.

Зрение (камера + YOLO/TensorRT) даёт курсовые углы людей; дальность берётся с лидара в
секторе вокруг этого угла. Получается `TargetObservation(range_m, bearing_rad)` — ровно
то, что нужно PersonFollower и FollowingBehavior.

Шов разделён, чтобы чистая конверсия проверялась без железа:
  * `PersonVision` — источник КУРСОВЫХ углов людей (0=прямо, +вправо). Реальная реализация
    оборачивает follow_me.Detector (YOLO) + камеру; в тестах — заглушка.
  * `_range_at_bearing` — медиана дальностей лидара в секторе (чистая функция).

Контракт: `greenhouse.sensing.TargetDetector`.
"""

from __future__ import annotations

import math
from typing import Protocol

from greenhouse.runtime.clock import Clock
from greenhouse.sensing.interfaces import LidarScan, LidarSource, TargetObservation


class PersonVision(Protocol):
    """Источник курсовых углов (рад, 0=прямо по курсу, +вправо) видимых людей."""

    def detect_bearings(self) -> list[float]: ...


def range_at_bearing(scan: LidarScan, bearing_rad: float, window_rad: float) -> float | None:
    """Медиана конечных дальностей лидара в секторе ±window/2 вокруг курса. None — нет возвратов."""
    half = window_rad / 2.0
    vals: list[float] = []
    for i, r in enumerate(scan.ranges_m):
        angle = scan.angle_min_rad + i * scan.angle_increment_rad
        if abs(_wrap(angle - bearing_rad)) <= half and math.isfinite(r) and r > 0.0:
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def _wrap(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class JetsonTargetDetector:
    """Реализует `greenhouse.sensing.TargetDetector`: зрение + дальнометрия лидаром."""

    def __init__(
        self,
        *,
        vision: PersonVision,
        lidar: LidarSource,
        clock: Clock,
        fusion_window_rad: float = math.radians(4.0),
    ) -> None:
        self._vision = vision
        self._lidar = lidar
        self._clock = clock
        self._window = fusion_window_rad

    def detect(self) -> TargetObservation | None:
        bearings = self._vision.detect_bearings()
        if not bearings:
            return None
        scan = self._lidar.read_scan()
        # Кандидаты с валидной лидарной дальностью; берём ближайшего.
        ranged = [
            (rng, b)
            for b in bearings
            if (rng := range_at_bearing(scan, b, self._window)) is not None
        ]
        if not ranged:
            return None  # # CALIBRATE: при желании — оценка дальности по высоте bbox (камера-only)
        rng, bearing = min(ranged, key=lambda rb: rb[0])
        return TargetObservation(range_m=rng, bearing_rad=bearing, stamp_s=self._clock.now_s())


def build_follow_me_vision(
    *,
    engine_path: str = "models/yolo11n.engine",
    camera_index: int = 0,
    hfov_rad: float = math.radians(70.0),  # CALIBRATE: реальный горизонтальный FOV объектива
) -> PersonVision:
    """Ленивая боевая реализация PersonVision поверх follow_me (YOLO + камера).

    Доступна только на Jetson, где установлен пакет follow_me и ultralytics/torch.
    Внедрите свою реализацию PersonVision в тестах/симуляции.
    """
    return _FollowMeVision(engine_path=engine_path, camera_index=camera_index, hfov_rad=hfov_rad)


class _FollowMeVision:
    """Обёртка follow_me.Detector (YOLO) + камера → курсовые углы людей. Только на Jetson."""

    def __init__(self, *, engine_path: str, camera_index: int, hfov_rad: float) -> None:
        try:
            import cv2  # type: ignore[import-not-found]
            from follow_me.config import FollowMeConfig  # type: ignore[import-not-found]
            from follow_me.detector import Detector  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - только вне Jetson
            raise ImportError(
                "Боевое зрение требует follow_me + ultralytics + cv2 (есть на Jetson). "
                "Внедрите свою реализацию PersonVision вне робота."
            ) from exc
        cfg = FollowMeConfig()
        cfg.engine_path = engine_path
        self._cfg = cfg
        self._hfov = hfov_rad
        self._detector = Detector(cfg)
        self._cap = cv2.VideoCapture(camera_index)

    def detect_bearings(self) -> list[float]:  # pragma: no cover - требует камеру/YOLO
        ok, frame = self._cap.read()
        if not ok:
            return []
        w = frame.shape[1]
        dets = self._detector.detect(frame)
        return [(d.cx / w - 0.5) * self._hfov for d in dets]
