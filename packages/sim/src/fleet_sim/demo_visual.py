"""Matplotlib window: walls, sparse lidar fan, robot and follow target."""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
from fleet_contracts.perception import LaserScan
from matplotlib.animation import FuncAnimation
from matplotlib.artist import Artist
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

from fleet_sim.demo_runner import ExploreFollowDemo


class _FollowTargetDragHandler:
    """Move the red follow-target marker by dragging within the axes."""

    _pick_radius_m: float

    def __init__(
        self,
        fig: Figure,
        ax: Axes,
        demo: ExploreFollowDemo,
        *,
        pick_radius_m: float = 0.45,
    ) -> None:
        self._fig = fig
        self._ax = ax
        self._demo = demo
        self._pick_radius_m = pick_radius_m
        self._dragging = False
        fig.canvas.mpl_connect("button_press_event", self._on_press)
        fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        fig.canvas.mpl_connect("button_release_event", self._on_release)

    def _event_xy_m(self, event: object) -> tuple[float, float] | None:
        xdata = getattr(event, "xdata", None)
        ydata = getattr(event, "ydata", None)
        if xdata is not None and ydata is not None:
            return (float(xdata), float(ydata))
        xe = getattr(event, "x", None)
        ye = getattr(event, "y", None)
        if xe is None or ye is None:
            return None
        inv = self._ax.transData.inverted()
        xd, yd = inv.transform((float(xe), float(ye)))
        return (float(xd), float(yd))

    def _on_press(self, event: object) -> None:
        if getattr(event, "inaxes", None) is not self._ax:
            return
        if getattr(event, "button", None) != 1:
            return
        xy = self._event_xy_m(event)
        if xy is None:
            return
        tx, ty = self._demo.state.target_xy_m
        if math.hypot(xy[0] - tx, xy[1] - ty) > self._pick_radius_m:
            return
        self._dragging = True

    def _on_motion(self, event: object) -> None:
        if not self._dragging:
            return
        xy = self._event_xy_m(event)
        if xy is None:
            return
        cx, cy = self._demo.world.clamp_point_to_interior(xy[0], xy[1])
        self._demo.state.set_target_xy(cx, cy)
        canvas = self._fig.canvas
        draw_idle = getattr(canvas, "draw_idle", None)
        if callable(draw_idle) and not isinstance(canvas, FigureCanvasAgg):
            draw_idle()

    def _on_release(self, _event: object) -> None:
        self._dragging = False


def _scan_ray_segments(
    scan: LaserScan,
    ox: float,
    oy: float,
    heading: float,
    stride: int,
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    beams: list[tuple[tuple[float, float], tuple[float, float]]] = []
    n = len(scan.ranges_m)
    idx = 0
    while idx < n:
        dist = scan.ranges_m[idx]
        if dist < scan.range_max_m * 0.997:
            ang = scan.angle_min_rad + float(idx) * scan.angle_increment_rad
            gx = ox + dist * math.cos(heading + ang)
            gy = oy + dist * math.sin(heading + ang)
            beams.append(((ox, oy), (gx, gy)))
        idx += stride
    return beams


class _RobotMarker:
    def __init__(self, ax: Axes) -> None:
        self.patch: Polygon = Polygon(
            [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
            closed=True,
            facecolor="#2e7d32",
            edgecolor="#1b5e20",
            linewidth=1.2,
            zorder=4,
        )
        ax.add_patch(self.patch)

    def set_pose(self, x: float, y: float, theta: float) -> None:
        nose_forward = 0.34
        back_inset = 0.11
        half_width = 0.16

        nx = x + nose_forward * math.cos(theta)
        ny = y + nose_forward * math.sin(theta)

        bx = x - back_inset * math.cos(theta)
        by = y - back_inset * math.sin(theta)

        lx = bx + half_width * (-math.sin(theta))
        ly = by + half_width * math.cos(theta)
        rx = bx - half_width * (-math.sin(theta))
        ry = by - half_width * math.cos(theta)

        self.patch.set_xy([(nx, ny), (lx, ly), (rx, ry)])


def main() -> None:
    demo = ExploreFollowDemo(max_steps=6500)

    fig, ax = plt.subplots(figsize=(11.0, 4.75))
    mgr = fig.canvas.manager
    if mgr is not None:
        try:
            mgr.set_window_title("fleet-sim — комнаты, лидар, цель (ЛКМ: перетащить)")
        except AttributeError:
            pass

    wall_segments = [((w.x0, w.y0), (w.x1, w.y1)) for w in demo.world.walls]
    walls_lc = LineCollection(wall_segments, colors="#37474f", linewidths=2.4)
    ax.add_collection(walls_lc)

    lidar_lc = LineCollection([], colors="#00acc1", linewidths=0.9, alpha=0.7)
    ax.add_collection(lidar_lc)

    robot_art = _RobotMarker(ax)

    tx0, ty0 = demo.state.target_xy_m
    target_scatter = ax.scatter(
        [tx0],
        [ty0],
        s=280,
        c="#ff1744",
        marker="P",
        zorder=5,
        edgecolors="#4a148c",
        linewidths=1,
        label="Цель следования",
    )

    route_line = Line2D(
        [],
        [],
        color="#5e35b1",
        linestyle=(0, (5, 3)),
        linewidth=2.0,
        alpha=0.88,
        zorder=2,
        label="Маршрут следования",
    )
    ax.add_line(route_line)

    pose0 = demo.state.robot_pose
    robot_art.set_pose(pose0.x_m, pose0.y_m, pose0.theta_rad)

    title = ax.text(
        0.02,
        0.97,
        "",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        fontfamily="monospace",
        color="#263238",
    )

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlim(-0.45, 12.95)
    ax.set_ylim(-0.4, 4.75)
    ax.set_xlabel("x, м")
    ax.set_ylabel("y, м")
    ax.grid(True, linestyle=":", alpha=0.45)
    legend = ax.legend(loc="upper right")
    legend.get_frame().set_alpha(0.92)

    def _frame_update(_idx: int) -> tuple[Artist, Artist, Artist, Artist, Artist]:
        demo.step(log_print=None)
        scan = demo.latest_scan_after_step()
        p = demo.state.robot_pose
        robot_art.set_pose(p.x_m, p.y_m, p.theta_rad)

        stride = max(2, len(scan.ranges_m) // 140) if scan is not None and scan.ranges_m else 4
        lidar_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        if scan is not None:
            lidar_segments = _scan_ray_segments(scan, p.x_m, p.y_m, p.theta_rad, stride=stride)

        lidar_lc.set_segments(lidar_segments)

        tgt = demo.state.target_xy_m
        target_scatter.set_offsets([[tgt[0], tgt[1]]])

        route_pts = demo.follow_route_polyline_m
        if len(route_pts) >= 2:
            route_line.set_data([p[0] for p in route_pts], [p[1] for p in route_pts])
        else:
            route_line.set_data([], [])

        title.set_text(
            (
                f"шаг={demo.step_index:<5}  режим={demo.phase:<10}  "
                f"x={p.x_m:.2f}  y={p.y_m:.2f}  θ°={math.degrees(p.theta_rad):.0f}"
            ),
        )

        return lidar_lc, robot_art.patch, route_line, target_scatter, title

    setattr(
        fig,
        "_fleet_sim_animation",
        FuncAnimation(
            fig,
            _frame_update,
            frames=demo.max_steps,
            interval=36,
            blit=False,
            repeat=False,
        ),
    )
    setattr(fig, "_fleet_sim_target_drag", _FollowTargetDragHandler(fig, ax, demo))
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
