"""Shared explore/follow FSM logic for headless demo and visualisation."""

from __future__ import annotations

import math
from collections.abc import Callable

from fleet_contracts.geometry import Pose2D
from fleet_contracts.perception import LaserScan

from fleet_sim.engine import simulation_step
from fleet_sim.exploration import RoomCentroidExplorer
from fleet_sim.follow_sync import twist_follow_point
from fleet_sim.mocks import (
    SimCameraSource,
    SimLidarSource,
    SimMotionController,
    SimPersonDetector,
    SimTruthLocalizer,
)
from fleet_sim.state import SimState
from fleet_sim.world import PolygonWorld, three_rooms_line_world


class ExploreFollowDemo:
    """One tick = control decision + rigid-body integration (point mass inside free space)."""

    dt_s: float = 0.05

    def __init__(
        self,
        *,
        world: PolygonWorld | None = None,
        robot0: Pose2D | None = None,
        target_xy_m: tuple[float, float] | None = None,
        max_steps: int = 4500,
    ) -> None:
        self.world = world or three_rooms_line_world()
        r0 = robot0 or Pose2D(x_m=1.6, y_m=2.0, theta_rad=0.0)
        target = target_xy_m or (10.2, 2.05)
        self.state = SimState(robot_pose=r0, target_xy_m=target)
        self.lidar_source = SimLidarSource(self.world, self.state)
        self.motion = SimMotionController(self.state)
        self._loc = SimTruthLocalizer(self.state)
        self.camera = SimCameraSource(self.state)
        self.detector = SimPersonDetector(
            self.state,
            fov_half_width_rad=math.radians(52.0),
            max_range_m=8.0,
        )
        self.explorer = RoomCentroidExplorer(self.world, self.lidar_source)
        self.max_steps = max_steps
        self.step_index = 0
        self.phase = "explore"
        self.follow_remaining = 0
        self.lost_budget = 0
        self.last_scan: LaserScan | None = None

    def latest_scan_after_step(self) -> LaserScan | None:
        """Scan from end of previous `step` (None until first step completes)."""

        return self.last_scan

    def step(self, *, log_print: Callable[[str], None] | None = None) -> None:
        """Advance one simulation sub-step."""

        pose = self.state.robot_pose
        frame = self.camera.acquire_frame()
        detections = self.detector.detect_persons(frame)

        if self.phase == "explore":
            self.explorer.mark_progress(pose)
            tw = self.explorer.compute_twist(pose)
            self.motion.send_twist(tw, watchdog_deadline_s=1.0)
            if detections:
                self.phase = "follow"
                self.follow_remaining = 900
                self.lost_budget = 40
                self.motion.halt_immediate(reason_code="phase_switch_explore_follow")
                if log_print is not None:
                    log_print(
                        "\t".join(
                            (
                                str(self.step_index),
                                "switch_to_follow",
                                f"{pose.x_m:.2f}",
                            )
                        )
                    )
        elif self.phase == "follow":
            if detections:
                self.lost_budget = 40
            else:
                self.lost_budget -= 1

            tx, ty = self.state.target_xy_m
            ftw = twist_follow_point(
                pose,
                tx,
                ty,
                max_linear_m_s=0.45,
                gain_omega=3.5,
                gain_linear=1.05,
            )
            self.motion.send_twist(ftw, watchdog_deadline_s=1.0)
            self.follow_remaining -= 1

            if self.follow_remaining <= 0 or self.lost_budget <= 0:
                self.phase = "explore"
                self.motion.halt_immediate(reason_code="phase_switch_follow_explore")
                if log_print is not None:
                    log_print("\t".join((str(self.step_index), "switch_to_explore")))

        simulation_step(self.world, self.state, self.dt_s)

        self.last_scan = self.lidar_source.read_scan()
        self.step_index += 1
