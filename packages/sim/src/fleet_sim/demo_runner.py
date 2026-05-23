"""Shared explore/follow FSM logic for headless demo and visualisation."""

from __future__ import annotations

import math
from collections.abc import Callable

from fleet_contracts.geometry import Pose2D
from fleet_contracts.perception import LaserScan

from fleet_sim.engine import simulation_step
from fleet_sim.exploration import RoomCentroidExplorer
from fleet_sim.follow_sync import twist_follow_point
from fleet_sim.grid_nav import plan_path_through_free_space
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
    """One tick = control decision + rigid-body integration (unicycle with inscribed-disk footprint)."""

    dt_s: float = 0.05
    follow_waypoint_reach_m: float = 0.36
    # Planning + collisions: inscribed disk (~triangular viz marker hull).
    robot_inscribed_radius_m: float = 0.20

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
        self.state = SimState(
            robot_pose=r0,
            target_xy_m=target,
            robot_inscribed_radius_m=self.robot_inscribed_radius_m,
        )
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
        self._follow_path_m: list[tuple[float, float]] = []
        self._follow_wp_idx = 0
        self._follow_goal_xy: tuple[float, float] | None = None

    def latest_scan_after_step(self) -> LaserScan | None:
        """Scan from end of previous `step` (None until first step completes)."""

        return self.last_scan

    @property
    def follow_route_polyline_m(self) -> tuple[tuple[float, float], ...]:
        """Vertices of the planned follow route (viz / logging); empty outside follow cache."""

        return tuple(self._follow_path_m)

    def _refresh_follow_route_if_needed(self, pose: Pose2D) -> None:
        tx_m, ty_m = self.state.target_xy_m
        goal = (tx_m, ty_m)
        goal_moved = self._follow_goal_xy is None or math.hypot(
            goal[0] - self._follow_goal_xy[0],
            goal[1] - self._follow_goal_xy[1],
        ) > 0.12
        missing_path = not self._follow_path_m
        if goal_moved or missing_path:
            self._follow_path_m = plan_path_through_free_space(
                self.world,
                (pose.x_m, pose.y_m),
                goal,
                robot_inscribed_radius_m=self.robot_inscribed_radius_m,
            )
            self._follow_wp_idx = 0
            self._follow_goal_xy = goal

    def _advance_follow_waypoint_index(self, pose: Pose2D) -> None:
        if not self._follow_path_m:
            return
        reach = self.follow_waypoint_reach_m
        while self._follow_wp_idx < len(self._follow_path_m) - 1:
            wx, wy = self._follow_path_m[self._follow_wp_idx]
            separation = math.hypot(pose.x_m - wx, pose.y_m - wy)
            if separation <= reach:
                self._follow_wp_idx += 1
            else:
                break

    def _active_follow_subgoal_xy(self, pose: Pose2D) -> tuple[float, float]:
        tx_m, ty_m = self.state.target_xy_m
        if not self._follow_path_m:
            return (tx_m, ty_m)
        self._advance_follow_waypoint_index(pose)
        wx, wy = self._follow_path_m[self._follow_wp_idx]
        return (wx, wy)

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
                self._follow_path_m.clear()
                self._follow_goal_xy = None
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

            self._refresh_follow_route_if_needed(pose)
            wx, wy = self._active_follow_subgoal_xy(pose)
            ftw = twist_follow_point(
                pose,
                wx,
                wy,
                max_linear_m_s=0.45,
                gain_omega=3.5,
                gain_linear=1.05,
            )
            self.motion.send_twist(ftw, watchdog_deadline_s=1.0)
            self.follow_remaining -= 1

            if self.follow_remaining <= 0 or self.lost_budget <= 0:
                self.phase = "explore"
                self._follow_path_m.clear()
                self._follow_goal_xy = None
                self.motion.halt_immediate(reason_code="phase_switch_follow_explore")
                if log_print is not None:
                    log_print("\t".join((str(self.step_index), "switch_to_explore")))

        simulation_step(self.world, self.state, self.dt_s)

        self.last_scan = self.lidar_source.read_scan()
        self.step_index += 1
