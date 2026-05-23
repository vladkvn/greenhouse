"""Console demo: centroid exploration transitions into synthetic person follow."""

from __future__ import annotations

import math

from fleet_sim.demo_runner import ExploreFollowDemo


def main() -> None:
    sim = ExploreFollowDemo()

    print("step", "phase", "x", "y", "theta_deg", sep="\t")

    def _log(line: str) -> None:
        print(line)

    for tick_index in range(sim.max_steps):
        if tick_index % 100 == 0:
            p = sim.state.robot_pose
            print(
                tick_index,
                sim.phase,
                f"{p.x_m:.2f}",
                f"{p.y_m:.2f}",
                f"{math.degrees(p.theta_rad):.1f}",
                sep="\t",
            )
        sim.step(log_print=_log)

    print("demo_finished", f"steps={sim.max_steps}", sep="\t")


if __name__ == "__main__":
    main()
