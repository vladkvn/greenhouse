"""OpenCV visualization: camera panel with people + a top-down LiDAR panel."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .config import FollowMeConfig
from .fusion import PersonTrack

# BGR colours
_GREEN = (0, 220, 0)
_RED = (0, 0, 255)
_YELLOW = (0, 220, 220)
_GREY = (90, 90, 90)
_WHITE = (240, 240, 240)
_FREE = (45, 30, 18)        # scanned free-space fill (dark blue-grey)
_WALL = (210, 170, 60)      # obstacle surfaces / walls (cyan-blue)
_OBJ = (70, 180, 255)       # object blobs (orange)


def _polar_px(cx: int, cy: int, deg: float, r_m: float, px_per_m: float) -> tuple[int, int]:
    """Polar (0deg = up/forward, clockwise) + range in metres -> pixel coords."""
    a = math.radians(deg)
    return (int(cx + r_m * px_per_m * math.sin(a)),
            int(cy - r_m * px_per_m * math.cos(a)))


def _cluster_scan(
    scan: np.ndarray, gap_deg: float, jump_m: float
) -> list[list[tuple[int, float]]]:
    """Group valid returns into objects, splitting on angular gaps or range jumps."""
    valid = [(deg, float(scan[deg])) for deg in range(360) if not math.isnan(scan[deg])]
    if not valid:
        return []
    clusters: list[list[tuple[int, float]]] = []
    cur = [valid[0]]
    for (pdeg, pdist), (deg, dist) in zip(valid, valid[1:]):
        if (deg - pdeg) <= gap_deg and abs(dist - pdist) <= jump_m:
            cur.append((deg, dist))
        else:
            clusters.append(cur)
            cur = [(deg, dist)]
    clusters.append(cur)
    # Merge across the 0/360 seam if the ends are adjacent.
    if len(clusters) > 1:
        f_deg, f_dist = clusters[0][0]
        l_deg, l_dist = clusters[-1][-1]
        if (360 - l_deg + f_deg) <= gap_deg and abs(l_dist - f_dist) <= jump_m:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()
    return clusters


class Visualizer:
    def __init__(self, cfg: FollowMeConfig):
        self._cfg = cfg
        self._size = cfg.lidar_panel_size

    # ----------------------------------------------------------------- public
    def render(
        self,
        frame: np.ndarray,
        tracks: list[PersonTrack],
        scan: np.ndarray,
        target: PersonTrack | None,
        fps: float,
    ) -> np.ndarray:
        cam_panel = self._draw_camera(frame.copy(), tracks, target)
        lidar_panel = self._draw_lidar(scan, tracks, target)

        # Match heights so hstack works regardless of camera resolution.
        h = cam_panel.shape[0]
        if lidar_panel.shape[0] != h:
            scale = h / lidar_panel.shape[0]
            lidar_panel = cv2.resize(
                lidar_panel, (int(lidar_panel.shape[1] * scale), h)
            )
        canvas = np.hstack([cam_panel, lidar_panel])
        cv2.putText(
            canvas, f"{fps:4.1f} FPS", (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, _WHITE, 2, cv2.LINE_AA,
        )
        return canvas

    # ---------------------------------------------------------------- camera
    def _draw_camera(
        self, frame: np.ndarray, tracks: list[PersonTrack], target: PersonTrack | None
    ) -> np.ndarray:
        h, w = frame.shape[:2]
        origin = (w // 2, h - 5)  # bottom-centre = robot heading reference
        for t in tracks:
            is_target = t is target
            color = _RED if is_target else _GREEN
            x1, y1, x2, y2 = map(int, (t.det.x1, t.det.y1, t.det.x2, t.det.y2))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2 if not is_target else 3)

            dist_txt = f"{t.distance_m:.2f} m" if t.distance_m is not None else "-- m"
            label = f"person {t.det.conf:.2f}"
            info = f"{dist_txt}  {t.cam_angle_deg:+.0f} deg"
            cv2.putText(frame, label, (x1, max(18, y1 - 24)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            cv2.putText(frame, info, (x1, max(36, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

            # Direction arrow from robot origin toward the person's bearing.
            self._draw_direction_arrow(frame, origin, t.cam_angle_deg, color)

        if target is not None:
            d = f"{target.distance_m:.2f} m" if target.distance_m is not None else "?"
            cv2.putText(
                frame, f"TARGET  angle={target.cam_angle_deg:+.1f} deg  dist={d}",
                (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, _RED, 2, cv2.LINE_AA,
            )
        return frame

    @staticmethod
    def _draw_direction_arrow(frame, origin, cam_angle_deg, color) -> None:
        length = 120
        ang = math.radians(cam_angle_deg)  # 0 = up/forward, +right
        tip = (
            int(origin[0] + length * math.sin(ang)),
            int(origin[1] - length * math.cos(ang)),
        )
        cv2.arrowedLine(frame, origin, tip, color, 3, tipLength=0.3)

    # ----------------------------------------------------------------- lidar
    def _draw_lidar(
        self, scan: np.ndarray, tracks: list[PersonTrack], target: PersonTrack | None
    ) -> np.ndarray:
        cfg = self._cfg
        size = self._size
        panel = np.zeros((size, size, 3), dtype=np.uint8)
        cx = cy = size // 2
        max_r = cfg.lidar_max_range_m
        px_per_m = (size / 2 - 20) / max_r

        # --- 1. Scanned free-space: fill the swept area out to each return (or max range).
        poly = [
            _polar_px(cx, cy, deg,
                      scan[deg] if not math.isnan(scan[deg]) else max_r, px_per_m)
            for deg in range(360)
        ]
        overlay = panel.copy()
        cv2.fillPoly(overlay, [np.array(poly, dtype=np.int32)], _FREE)
        cv2.addWeighted(overlay, 0.9, panel, 0.1, 0, panel)

        # --- 2. Range rings + forward axis (drawn over the free-space fill).
        for r in (2, 4, 6, 8, 12):
            if r > max_r:
                continue
            cv2.circle(panel, (cx, cy), int(r * px_per_m), _GREY, 1, cv2.LINE_AA)
            cv2.putText(panel, f"{r}m", (cx + int(r * px_per_m) - 22, cy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, _GREY, 1, cv2.LINE_AA)
        cv2.line(panel, (cx, cy), (cx, 20), _GREY, 1, cv2.LINE_AA)
        cv2.putText(panel, "fwd", (cx + 4, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, _GREY, 1, cv2.LINE_AA)

        # --- 3. Cluster returns into objects; draw walls + filled blobs.
        clusters = _cluster_scan(scan, cfg.map_cluster_gap_deg, cfg.map_cluster_jump_m)
        overlay = panel.copy()
        for cluster in clusters:
            pts = np.array(
                [_polar_px(cx, cy, deg, dist, px_per_m) for deg, dist in cluster],
                dtype=np.int32,
            )
            is_person = self._cluster_is_person(cluster, tracks)
            wall_col = _RED if is_person else _WALL
            # Connect adjacent returns -> surface / wall outline.
            if len(pts) >= 2:
                cv2.polylines(panel, [pts], False, wall_col, 2, cv2.LINE_AA)
            # Larger clusters become filled object blobs (convex hull).
            if len(cluster) >= cfg.map_min_cluster_pts and len(pts) >= 3:
                hull = cv2.convexHull(pts)
                fill = _RED if is_person else _OBJ
                cv2.fillPoly(overlay, [hull], fill)
                cv2.polylines(panel, [hull], True, fill, 1, cv2.LINE_AA)
                # Label with the object's nearest distance.
                near = min(d for _deg, d in cluster)
                mx, my = int(pts[:, 0].mean()), int(pts[:, 1].mean())
                tag = ("PERSON " if is_person else "") + f"{near:.1f}m"
                cv2.putText(panel, tag, (mx - 18, my - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            _RED if is_person else _WHITE, 1, cv2.LINE_AA)
            else:
                for p in pts:
                    cv2.circle(panel, tuple(int(v) for v in p), 2, _WHITE, -1)
        cv2.addWeighted(overlay, 0.35, panel, 0.65, 0, panel)

        # --- 4. Bearing ray to the follow target.
        if target is not None:
            a = math.radians(target.lidar_angle_deg)
            end = (int(cx + (size / 2) * math.sin(a)), int(cy - (size / 2) * math.cos(a)))
            cv2.line(panel, (cx, cy), end, _RED, 1, cv2.LINE_AA)

        # --- 5. Robot at the centre, pointing forward.
        cv2.drawMarker(panel, (cx, cy), _GREEN, cv2.MARKER_TRIANGLE_UP, 16, 2)
        return panel

    def _cluster_is_person(
        self, cluster: list[tuple[int, float]], tracks: list[PersonTrack]
    ) -> bool:
        """True if a detected person's (angle, distance) falls inside this cluster."""
        degs = [d for d, _ in cluster]
        lo, hi = min(degs), max(degs)
        for t in tracks:
            if t.distance_m is None:
                continue
            ang = t.lidar_angle_deg % 360
            in_span = lo <= ang <= hi or (hi - lo > 180 and (ang >= hi or ang <= lo))
            near = min(d for _deg, d in cluster)
            if in_span and abs(t.distance_m - near) <= self._cfg.map_person_match_m:
                return True
        return False
