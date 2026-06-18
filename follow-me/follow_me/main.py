"""Follow-me perception entry point.

Pipeline per frame: grab camera -> YOLO person detection (GPU/TensorRT) ->
fuse with LiDAR ranging -> visualize -> log the (angle, distance) steering command.

Run:  python -m follow_me.main
Quit: press 'q' in the window (or Ctrl-C in headless mode).
"""

from __future__ import annotations

import argparse
import logging
import time

import cv2

from .config import FollowMeConfig
from .detector import Detector
from .fusion import fuse, select_target
from .lidar import LidarThread
from .motor import Esp32Motor, compute_drive
from .visualizer import Visualizer

log = logging.getLogger("follow_me")


def open_camera(cfg: FollowMeConfig) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(cfg.camera_index, cv2.CAP_V4L2)
    # MJPG keeps USB bandwidth sane at HD resolutions.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.frame_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.frame_h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.camera_index}")
    # Honour whatever the driver actually granted.
    cfg.frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or cfg.frame_w
    cfg.frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or cfg.frame_h
    log.info("Camera opened at %dx%d", cfg.frame_w, cfg.frame_h)
    return cap


def run(cfg: FollowMeConfig) -> None:
    lidar = LidarThread(cfg.lidar_port, cfg.lidar_baud, cfg.lidar_max_range_m)
    lidar.start()
    if not lidar.wait_until_ready(timeout=10.0):
        log.warning("LiDAR not ready yet -- continuing; distances appear once it spins up.")

    detector = Detector(cfg)
    visualizer = Visualizer(cfg)
    cap = open_camera(cfg)

    motor = Esp32Motor(cfg.esp32_ip, cfg.esp32_port) if cfg.enable_drive else None
    if motor is not None:
        log.info("Drive ENABLED -> ESP32 %s:%d (stop at %.2f m)",
                 cfg.esp32_ip, cfg.esp32_port, cfg.stop_distance_m)
    else:
        log.info("Drive disabled (perception only). Pass --drive to move the robot.")

    fps = 0.0
    last = time.monotonic()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                log.warning("Frame grab failed")
                continue

            detections = detector.detect(frame)
            tracks = fuse(detections, lidar, cfg)
            target = select_target(tracks)

            # Turn the target into a drive command and send it to the ESP32.
            left, right, drive_status = compute_drive(target, cfg)
            if motor is not None:
                if left == 0 and right == 0:
                    motor.stop()
                else:
                    motor.drive(left, right)

            if target is not None:
                dist = (
                    f"{target.distance_m:.2f}m" if target.distance_m is not None else "?"
                )
                log.info("STEER angle=%+.1f deg dist=%s | %s",
                         target.cam_angle_deg, dist, drive_status)

            now = time.monotonic()
            dt = now - last
            last = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

            if cfg.show_window:
                scan = lidar.get_scan()
                canvas = visualizer.render(frame, tracks, scan, target, fps)
                cv2.imshow(cfg.window_name, canvas)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        if motor is not None:
            motor.stop()      # leave the robot stopped on exit
            motor.close()
        lidar.stop()
        cap.release()
        cv2.destroyAllWindows()
        lidar.join(timeout=3.0)
        log.info("Shut down cleanly")


def parse_args() -> FollowMeConfig:
    cfg = FollowMeConfig()
    p = argparse.ArgumentParser(description="Follow-me perception (camera + LiDAR)")
    p.add_argument("--camera-index", type=int, default=cfg.camera_index)
    p.add_argument("--lidar-port", default=cfg.lidar_port)
    p.add_argument("--engine", dest="engine_path", default=cfg.engine_path)
    p.add_argument("--hfov", dest="camera_hfov_deg", type=float, default=cfg.camera_hfov_deg)
    p.add_argument("--offset", dest="cam_to_lidar_offset_deg", type=float,
                   default=cfg.cam_to_lidar_offset_deg)
    p.add_argument("--flip", dest="lidar_flip", action="store_true", default=cfg.lidar_flip)
    p.add_argument("--no-window", dest="show_window", action="store_false",
                   default=cfg.show_window)
    p.add_argument("--drive", dest="enable_drive", action="store_true",
                   default=cfg.enable_drive, help="send motor commands to the ESP32")
    p.add_argument("--esp32", dest="esp32_ip", default=cfg.esp32_ip,
                   help="ESP32 motor module IP")
    args = p.parse_args()
    for k, v in vars(args).items():
        setattr(cfg, k, v)
    return cfg


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    run(parse_args())


if __name__ == "__main__":
    main()
