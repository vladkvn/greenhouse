"""Tests for the LiDAR 2D-map clustering. Skipped where OpenCV isn't importable."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("cv2", exc_type=ImportError)  # skip on hosts without OpenCV

from follow_me.visualizer import _cluster_scan, _polar_px  # noqa: E402


def _empty() -> np.ndarray:
    return np.full(360, np.nan, dtype=np.float32)


def test_no_returns_yields_no_clusters():
    assert _cluster_scan(_empty(), gap_deg=6.0, jump_m=0.35) == []


def test_two_objects_split_by_angular_gap():
    scan = _empty()
    scan[10:15] = 2.0          # object A
    scan[100:106] = 3.0        # object B (far in angle)
    clusters = _cluster_scan(scan, gap_deg=6.0, jump_m=0.35)
    assert len(clusters) == 2
    assert {len(c) for c in clusters} == {5, 6}


def test_split_on_range_jump_same_angle_band():
    scan = _empty()
    scan[20:23] = 1.5
    scan[23:26] = 3.0          # sudden range jump -> different object
    clusters = _cluster_scan(scan, gap_deg=6.0, jump_m=0.35)
    assert len(clusters) == 2


def test_wraparound_merges_across_zero():
    scan = _empty()
    scan[357:360] = 2.0
    scan[0:3] = 2.0            # contiguous across the 0/360 seam
    clusters = _cluster_scan(scan, gap_deg=6.0, jump_m=0.35)
    assert len(clusters) == 1
    assert len(clusters[0]) == 6


def test_polar_px_forward_is_up():
    px, py = _polar_px(100, 100, deg=0, r_m=2.0, px_per_m=10.0)
    assert px == 100          # straight ahead -> no x offset
    assert py == 80           # 2 m * 10 px/m up from centre
