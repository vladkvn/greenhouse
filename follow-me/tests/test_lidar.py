"""Tests for LiDAR binning and angular lookup (no hardware, no rplidar import)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from follow_me.lidar import LidarThread


def _make(max_range_m: float = 12.0) -> LidarThread:
    return LidarThread(port="/dev/null", baud=115200, max_range_m=max_range_m)


def test_ingest_bins_by_degree_and_converts_to_metres():
    lt = _make()
    # (quality, angle_deg, distance_mm)
    lt._ingest([(15, 0.2, 2000.0), (15, 90.6, 4000.0)])
    scan = lt.get_scan()
    assert scan[0] == pytest.approx(2.0)    # 0.2 deg rounds to bin 0
    assert scan[91] == pytest.approx(4.0)   # 90.6 deg rounds to bin 91
    assert math.isnan(scan[45])             # nothing there


def test_ingest_drops_out_of_range_returns():
    lt = _make(max_range_m=12.0)
    lt._ingest([(15, 10.0, 0.0), (15, 20.0, 99000.0)])  # zero and >12m
    scan = lt.get_scan()
    assert math.isnan(scan[10])
    assert math.isnan(scan[20])


def test_distance_at_returns_median_in_window():
    lt = _make()
    lt._ingest([(15, 99.0, 3000.0), (15, 100.0, 5000.0), (15, 101.0, 4000.0)])
    # +/- 2 deg window around 100 covers bins 98..102 -> median of {3,5,4} = 4
    assert lt.distance_at(100.0, window_deg=4.0) == pytest.approx(4.0)


def test_distance_at_wraps_around_zero():
    lt = _make()
    lt._ingest([(15, 359.0, 2000.0), (15, 1.0, 2000.0)])
    # window around 0 must include 359 and 1
    assert lt.distance_at(0.0, window_deg=4.0) == pytest.approx(2.0)


def test_distance_at_none_when_no_returns():
    lt = _make()
    assert lt.distance_at(180.0, window_deg=4.0) is None


def test_get_scan_returns_a_copy():
    lt = _make()
    lt._ingest([(15, 0.0, 1000.0)])
    scan = lt.get_scan()
    scan[0] = 999.0
    assert lt.get_scan()[0] == pytest.approx(1.0)  # internal state untouched
