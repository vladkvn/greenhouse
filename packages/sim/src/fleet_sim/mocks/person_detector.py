"""Geometric visibility model for a single trackable point target."""

from __future__ import annotations

import math

from fleet_contracts.follow import BoundingBoxNormalized, PersonObservation
from fleet_contracts.perception import CameraFrame

from fleet_sim.physics import wrap_pi
from fleet_sim.state import SimState


class SimPersonDetector:
    """Target is a point mass; detection if inside front FOV cone and range limit."""

    TRACK_ID = "sim_target"

    def __init__(self, state: SimState, *, fov_half_width_rad: float, max_range_m: float) -> None:
        self._state = state
        self._fov_half = fov_half_width_rad
        self._max_range_m = max_range_m

    def detect_persons(self, frame: CameraFrame) -> tuple[PersonObservation, ...]:
        _ = frame
        tx, ty = self._state.target_xy_m
        rx = self._state.robot_pose.x_m
        ry = self._state.robot_pose.y_m
        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-6 or dist > self._max_range_m:
            return ()
        bearing_global = math.atan2(dy, dx)
        rel = wrap_pi(bearing_global - self._state.robot_pose.theta_rad)
        if abs(rel) > self._fov_half:
            return ()
        half_w = min(0.12, 0.8 / max(dist, 0.4))
        half_h = min(0.18, 1.0 / max(dist, 0.4))
        cx = 0.5 + (rel / self._fov_half) * 0.35
        cy = 0.52
        xmin = max(0.0, cx - half_w)
        xmax = min(1.0, cx + half_w)
        ymin = max(0.0, cy - half_h)
        ymax = min(1.0, cy + half_h)
        if xmax <= xmin or ymax <= ymin:
            return ()
        obs = PersonObservation(
            track_id=self.TRACK_ID,
            confidence=min(1.0, 3.0 / max(dist, 0.5)),
            bbox=BoundingBoxNormalized(xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax),
        )
        return (obs,)
