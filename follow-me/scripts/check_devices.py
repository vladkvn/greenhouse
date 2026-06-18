"""Quick hardware sanity check: camera frame grab + LiDAR health/scan.

    python scripts/check_devices.py
    python scripts/check_devices.py --camera-index 0 --lidar-port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import glob
import sys


def check_camera(index: int) -> bool:
    import cv2

    print(f"[camera] video devices: {sorted(glob.glob('/dev/video*'))}")
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[camera] FAILED to open index {index}")
        return False
    ok, frame = cap.read()
    cap.release()
    if not ok:
        print("[camera] opened but could not read a frame")
        return False
    print(f"[camera] OK -- frame shape {frame.shape}")
    return True


def check_lidar(port: str, baud: int) -> bool:
    print(f"[lidar] serial devices: {sorted(glob.glob('/dev/ttyUSB*'))}")
    try:
        from rplidar import RPLidar
    except ImportError:
        print("[lidar] rplidar not installed (pip install rplidar-roboticia)")
        return False
    lidar = None
    try:
        lidar = RPLidar(port, baudrate=baud)
        print(f"[lidar] info:   {lidar.get_info()}")
        print(f"[lidar] health: {lidar.get_health()}")
        for i, scan in enumerate(lidar.iter_scans()):
            print(f"[lidar] OK -- first scan has {len(scan)} points")
            break
        return True
    except Exception as exc:  # noqa: BLE001 - diagnostic tool, report anything
        print(f"[lidar] FAILED: {exc}")
        return False
    finally:
        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:  # noqa: BLE001
                pass


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--camera-index", type=int, default=0)
    p.add_argument("--lidar-port", default="/dev/ttyUSB0")
    p.add_argument("--lidar-baud", type=int, default=115200)
    args = p.parse_args()

    cam_ok = check_camera(args.camera_index)
    lidar_ok = check_lidar(args.lidar_port, args.lidar_baud)
    print(f"\nSummary: camera={'OK' if cam_ok else 'FAIL'}  "
          f"lidar={'OK' if lidar_ok else 'FAIL'}")
    return 0 if (cam_ok and lidar_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
