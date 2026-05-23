"""Room-centroid waypoint tour with lidar-based speed limiting."""

from __future__ import annotations

import math

from fleet_contracts.geometry import Pose2D, Twist2D
from fleet_contracts.perception import LidarSource

from fleet_sim.lidar_raycast import min_range_forward_cone
from fleet_sim.physics import wrap_pi
from fleet_sim.world import PolygonWorld


class RoomCentroidExplorer:
    """Visit each room centroid in label order; slows when obstacles ahead."""

    reach_radius_m: float = 0.45
    cone_half_width_rad: float = math.radians(42.0)
    max_linear_m_s: float = 0.52
    gain_omega: float = 2.2
    gain_linear: float = 0.95
    slowdown_range_m: float = 0.55
    halt_range_m: float = 0.12

    def __init__(self, world: PolygonWorld, lidar: LidarSource) -> None:
        ordered = sorted(world.rooms, key=lambda r: r.label)
        centroids = (r.centroid() for r in ordered)
        self._goals = tuple(Pose2D(x_m=c[0], y_m=c[1], theta_rad=0.0) for c in centroids)
        self._goal_index = 0
        self._lidar = lidar
        self._visited_centroids: set[int] = set()

    def goal_reached(self, pose: Pose2D) -> bool:
        goal = self.active_goal()
        dist = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
        return dist < self.reach_radius_m

    def active_goal(self) -> Pose2D:
        return self._goals[self._goal_index]

    def mark_progress(self, pose: Pose2D) -> None:
        if self.goal_reached(pose):
            self._visited_centroids.add(self._goal_index)
            self._goal_index = (self._goal_index + 1) % len(self._goals)

    def all_centroids_visited_once(self) -> bool:
        return len(self._visited_centroids) >= len(self._goals)

    def compute_twist(self, pose: Pose2D) -> Twist2D:
        goal = self.active_goal()
        dx = goal.x_m - pose.x_m
        dy = goal.y_m - pose.y_m
        heading_to_goal = math.atan2(dy, dx)
        angular_error = wrap_pi(heading_to_goal - pose.theta_rad)

        scan = self._lidar.read_scan()
        min_fwd = min_range_forward_cone(scan, cone_half_width_rad=self.cone_half_width_rad)
        if min_fwd < self.halt_range_m:
            return Twist2D(linear_x_m_s=0.0, angular_z_rad_s=0.85 * angular_error)

        speed_scale = min(1.0, max(0.0, (min_fwd - self.halt_range_m) / self.slowdown_range_m))

        linear = min(
            self.max_linear_m_s,
            speed_scale * self.gain_linear * math.hypot(dx, dy),
        )
        omega = self.gain_omega * angular_error
        return Twist2D(linear_x_m_s=linear, angular_z_rad_s=omega)
