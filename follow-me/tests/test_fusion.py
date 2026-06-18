"""Tests for the camera<->LiDAR geometry and target selection (no hardware needed)."""

from __future__ import annotations

import math

import pytest

from follow_me.config import FollowMeConfig
from follow_me.detector import Detection
from follow_me.fusion import (
    bbox_to_camera_angle,
    camera_angle_to_lidar_angle,
    fuse,
    select_target,
)


def test_bbox_center_is_zero_bearing():
    assert bbox_to_camera_angle(640, 1280, 70.0) == pytest.approx(0.0)


def test_bbox_left_is_negative_right_is_positive():
    left = bbox_to_camera_angle(0, 1280, 70.0)
    right = bbox_to_camera_angle(1280, 1280, 70.0)
    assert left == pytest.approx(-35.0)
    assert right == pytest.approx(+35.0)


def test_camera_angle_to_lidar_wraps_and_offsets():
    # Coaxial, no flip: forward bearing maps straight through.
    assert camera_angle_to_lidar_angle(0.0, 0.0, False) == pytest.approx(0.0)
    # A left bearing (-10) wraps to 350 on the 0..360 LiDAR frame.
    assert camera_angle_to_lidar_angle(-10.0, 0.0, False) == pytest.approx(350.0)
    # Offset is additive and wraps.
    assert camera_angle_to_lidar_angle(5.0, 358.0, False) == pytest.approx(3.0)


def test_camera_angle_flip_inverts_sign():
    assert camera_angle_to_lidar_angle(10.0, 0.0, True) == pytest.approx(350.0)


class _FakeLidar:
    """Stand-in for LidarThread.distance_at."""

    def __init__(self, mapping: dict[int, float]):
        self._mapping = mapping

    def distance_at(self, angle_deg: float, window_deg: float):
        return self._mapping.get(round(angle_deg) % 360)


def test_fuse_attaches_distance_and_angle():
    cfg = FollowMeConfig(frame_w=1280, camera_hfov_deg=70.0)
    det = Detection(600, 100, 680, 500, 0.9)  # centre x = 640 -> 0 deg
    lidar = _FakeLidar({0: 2.5})
    tracks = fuse([det], lidar, cfg)
    assert len(tracks) == 1
    t = tracks[0]
    assert t.cam_angle_deg == pytest.approx(0.0)
    assert t.lidar_angle_deg == pytest.approx(0.0)
    assert t.distance_m == pytest.approx(2.5)


def test_fuse_none_distance_when_no_return():
    cfg = FollowMeConfig()
    det = Detection(0, 0, 100, 400, 0.8)
    lidar = _FakeLidar({})  # nothing at any angle
    tracks = fuse([det], lidar, cfg)
    assert tracks[0].distance_m is None


def test_select_target_prefers_nearest_ranged():
    cfg = FollowMeConfig()
    near = Detection(600, 0, 680, 400, 0.9)
    far = Detection(0, 0, 200, 400, 0.9)  # bigger bbox but farther
    lidar = _FakeLidar({
        round(camera_angle_to_lidar_angle(
            bbox_to_camera_angle(near.cx, cfg.frame_w, cfg.camera_hfov_deg), 0.0, False)): 1.5,
        round(camera_angle_to_lidar_angle(
            bbox_to_camera_angle(far.cx, cfg.frame_w, cfg.camera_hfov_deg), 0.0, False)) % 360: 5.0,
    })
    tracks = fuse([near, far], lidar, cfg)
    target = select_target(tracks)
    assert target is not None
    assert target.distance_m == pytest.approx(1.5)


def test_select_target_falls_back_to_largest_bbox():
    cfg = FollowMeConfig()
    small = Detection(600, 0, 640, 200, 0.9)
    big = Detection(0, 0, 300, 600, 0.9)
    lidar = _FakeLidar({})  # no LiDAR returns
    tracks = fuse([small, big], lidar, cfg)
    target = select_target(tracks)
    assert target is not None
    assert target.det is big


def test_select_target_none_when_empty():
    assert select_target([]) is None
