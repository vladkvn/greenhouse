"""Background RPLiDAR reader.

Spins the LiDAR in its own thread and keeps the latest full 360-degree scan as a
numpy array of 360 one-degree bins (metres, NaN where there is no return). The main
loop reads a thread-safe copy and queries distance at any angle.
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

log = logging.getLogger(__name__)


def _import_rplidar():
    """Import rplidar lazily so the pure binning logic stays testable off-Jetson."""
    try:
        from rplidar import RPLidar, RPLidarException
    except ImportError as exc:  # pragma: no cover - non-Jetson dev hosts
        raise ImportError(
            "rplidar not installed. On the Jetson run: pip install rplidar-roboticia"
        ) from exc
    return RPLidar, RPLidarException


class LidarThread(threading.Thread):
    """Continuously reads scans from an RPLiDAR A1 into a shared 360-bin array."""

    def __init__(self, port: str, baud: int, max_range_m: float,
                 mount_offset_deg: int = 0):
        super().__init__(daemon=True)
        self._port = port
        self._baud = baud
        self._max_range_m = max_range_m
        # Разворот скана: бин = (физ.угол + offset) % 360, чтобы бин 0 = перёд робота.
        self._mount_offset = int(round(mount_offset_deg)) % 360
        self._lidar = None  # set in run() once rplidar is imported
        self._exc_types: tuple[type[Exception], ...] = (OSError,)
        self._lock = threading.Lock()
        self._bins = np.full(360, np.nan, dtype=np.float32)
        self._stop_event = threading.Event()
        self._connected = threading.Event()

    # ------------------------------------------------------------------ public
    def run(self) -> None:
        RPLidar, RPLidarException = _import_rplidar()
        self._exc_types = (RPLidarException, OSError)
        while not self._stop_event.is_set():
            try:
                self._lidar = RPLidar(self._port, baudrate=self._baud)
                info = self._lidar.get_info()
                health = self._lidar.get_health()
                log.info("LiDAR connected: %s | health=%s", info, health)
                self._connected.set()
                self._read_loop()
            except self._exc_types as exc:
                log.warning("LiDAR error: %s -- reconnecting in 2s", exc)
                self._connected.clear()
                self._safe_teardown()
                if self._stop_event.wait(2.0):
                    break
        self._safe_teardown()

    def get_scan(self) -> np.ndarray:
        """Return a copy of the latest 360-bin scan (metres, NaN = no return)."""
        with self._lock:
            return self._bins.copy()

    def _window_vals(self, angle_deg: float, window_deg: float) -> np.ndarray:
        half = window_deg / 2.0
        center = angle_deg % 360.0
        # Build the integer degree indices inside the window, wrapping around 360.
        lo = int(np.floor(center - half))
        hi = int(np.ceil(center + half))
        idx = np.arange(lo, hi + 1) % 360
        with self._lock:
            vals = self._bins[idx]
        return vals[~np.isnan(vals)]

    def distance_at(self, angle_deg: float, window_deg: float) -> float | None:
        """Median distance (metres) of returns within +/- window_deg/2 of angle_deg."""
        vals = self._window_vals(angle_deg, window_deg)
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def nearest_at(self, angle_deg: float, window_deg: float) -> float | None:
        """Nearest return (metres) within +/- window_deg/2 of angle_deg.

        For 'distance to the person' this beats the median: the person is the
        closest object along their bearing, while a wall behind them sits farther.
        A wider window tolerates the camera<->LiDAR parallax at range.
        """
        vals = self._window_vals(angle_deg, window_deg)
        if vals.size == 0:
            return None
        return float(np.min(vals))

    def wait_until_ready(self, timeout: float = 10.0) -> bool:
        return self._connected.wait(timeout)

    def stop(self) -> None:
        self._stop_event.set()

    # ----------------------------------------------------------------- private
    def _read_loop(self) -> None:
        assert self._lidar is not None
        # iter_scans yields lists of (quality, angle_deg, distance_mm)
        for scan in self._lidar.iter_scans():
            if self._stop_event.is_set():
                break
            self._ingest(scan)

    def _ingest(self, scan: list[tuple[int, float, float]]) -> None:
        # Decay old data so stale bins eventually clear when the world changes.
        new_bins = np.full(360, np.nan, dtype=np.float32)
        for _quality, angle, dist_mm in scan:
            d_m = dist_mm / 1000.0
            if d_m <= 0.0 or d_m > self._max_range_m:
                continue
            new_bins[(int(round(angle)) + self._mount_offset) % 360] = d_m
        with self._lock:
            self._bins = new_bins

    def _safe_teardown(self) -> None:
        if self._lidar is None:
            return
        try:
            self._lidar.stop()
            self._lidar.stop_motor()
            self._lidar.disconnect()
        except self._exc_types as exc:  # pragma: no cover - best effort
            log.debug("LiDAR teardown error (ignored): %s", exc)
        finally:
            self._lidar = None
            time.sleep(0.1)
