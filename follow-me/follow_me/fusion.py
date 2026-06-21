"""Fuse camera detections with LiDAR ranging.

Geometry: a person's horizontal position in the frame gives a bearing relative to the
camera optical axis (negative = left, positive = right). Mapping that bearing onto the
LiDAR's polar frame (0deg = forward) lets us read the distance at the person's angle.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import FollowMeConfig
from .detector import Detection
from .lidar import LidarThread


@dataclass
class PersonTrack:
    det: Detection
    cam_angle_deg: float          # bearing relative to camera axis (- left / + right)
    lidar_angle_deg: float        # angle queried on the LiDAR (0..360)
    distance_m: float | None      # None when the LiDAR has no return at that angle


def bbox_to_camera_angle(cx: float, frame_w: float, hfov_deg: float) -> float:
    """Map a bbox horizontal centre to a bearing in degrees (- left / + right)."""
    return (cx / frame_w - 0.5) * hfov_deg


def camera_angle_to_lidar_angle(cam_angle: float, offset_deg: float, flip: bool) -> float:
    """Convert a camera bearing to a LiDAR angle (0deg forward, wrapped to 0..360)."""
    a = -cam_angle if flip else cam_angle
    return (a + offset_deg) % 360.0


def fuse(
    detections: list[Detection],
    lidar: LidarThread,
    cfg: FollowMeConfig,
) -> list[PersonTrack]:
    tracks: list[PersonTrack] = []
    for det in detections:
        cam_angle = bbox_to_camera_angle(det.cx, cfg.frame_w, cfg.camera_hfov_deg)
        lidar_angle = camera_angle_to_lidar_angle(
            cam_angle, cfg.cam_to_lidar_offset_deg, cfg.lidar_flip
        )
        distance = lidar.nearest_at(lidar_angle, cfg.fusion_window_deg)
        tracks.append(PersonTrack(det, cam_angle, lidar_angle, distance))
    return tracks


def select_target(tracks: list[PersonTrack]) -> PersonTrack | None:
    """Pick the follow-me target: nearest by LiDAR, else the largest bbox.

    This is the integration point for future motion control: the returned track's
    (cam_angle_deg, distance_m) is the steering command.
    """
    if not tracks:
        return None
    ranged = [t for t in tracks if t.distance_m is not None]
    if ranged:
        return min(ranged, key=lambda t: t.distance_m)  # type: ignore[arg-type]
    return max(tracks, key=lambda t: t.det.area)
