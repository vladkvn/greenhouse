"""Console demo: centroid exploration transitions into synthetic person follow."""

from __future__ import annotations

import math

from fleet_contracts.geometry import Pose2D

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
from fleet_sim.world import three_rooms_line_world


def main() -> None:
    world = three_rooms_line_world()
    robot0 = Pose2D(x_m=1.6, y_m=2.0, theta_rad=0.0)
    target_stationary = (10.2, 2.05)
    state = SimState(robot_pose=robot0, target_xy_m=target_stationary)

    lidar = SimLidarSource(world, state)
    motion = SimMotionController(state)
    _loc = SimTruthLocalizer(state)
    camera = SimCameraSource(state)
    detector = SimPersonDetector(
        state,
        fov_half_width_rad=math.radians(52.0),
        max_range_m=8.0,
    )

    explorer = RoomCentroidExplorer(world, lidar)

    dt_s = 0.05
    max_steps = 4500

    phase = "explore"
    follow_remaining = 0
    lost_budget = 0

    print("step", "phase", "x", "y", "theta_deg", sep="\t")

    for step_idx in range(max_steps):
        pose = state.robot_pose
        frame = camera.acquire_frame()
        detections = detector.detect_persons(frame)

        if phase == "explore":
            explorer.mark_progress(pose)
            tw = explorer.compute_twist(pose)
            motion.send_twist(tw, watchdog_deadline_s=1.0)
            if detections:
                phase = "follow"
                follow_remaining = 900
                lost_budget = 40
                motion.halt_immediate(reason_code="phase_switch_explore_follow")
                print(step_idx, "switch_to_follow", f"{pose.x_m:.2f}", sep="\t")
        elif phase == "follow":
            if detections:
                lost_budget = 40
            else:
                lost_budget -= 1

            tx, ty = state.target_xy_m
            ftw = twist_follow_point(
                pose,
                tx,
                ty,
                max_linear_m_s=0.45,
                gain_omega=3.5,
                gain_linear=1.05,
            )
            motion.send_twist(ftw, watchdog_deadline_s=1.0)
            follow_remaining -= 1

            if follow_remaining <= 0 or lost_budget <= 0:
                phase = "explore"
                motion.halt_immediate(reason_code="phase_switch_follow_explore")
                print(step_idx, "switch_to_explore", sep="\t")

        simulation_step(world, state, dt_s)

        if step_idx % 100 == 0:
            print(
                step_idx,
                phase,
                f"{pose.x_m:.2f}",
                f"{pose.y_m:.2f}",
                f"{math.degrees(pose.theta_rad):.1f}",
                sep="\t",
            )

    print("demo_finished", f"steps={max_steps}", sep="\t")


if __name__ == "__main__":
    main()
