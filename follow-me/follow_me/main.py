"""Follow-me perception entry point.

Pipeline per frame: grab camera -> YOLO person detection (GPU/TensorRT) ->
fuse with LiDAR ranging -> visualize -> log the (angle, distance) steering command.

Run:  python -m follow_me.main
Quit: press 'q' in the window (or Ctrl-C in headless mode).
"""

from __future__ import annotations

import argparse
import logging
import math
import time

import cv2
import numpy as np

from .config import FollowMeConfig
from .detector import Detector
from .fusion import fuse, select_target
from .imu import ImuReader
from .lidar import LidarThread
from .motor import Esp32Motor, compute_drive
from .navigation import Navigator
from .slam import SlamMapper
from .visualizer import Visualizer
from .webserver import (
    MODE_LABELS,
    SharedState,
    WebServer,
    annotate,
    encode_jpeg,
)

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


def _ramp(cur: int, target: int, step: int) -> int:
    """Сдвинуть cur к target не более чем на step (плавный старт/стоп моторов)."""
    if target > cur:
        return min(target, cur + step)
    if target < cur:
        return max(target, cur - step)
    return cur


def render_map_png(slam: SlamMapper, navigator: Navigator):
    """Карта SLAM как PNG: серый occupancy + робот (поза/курс) + путь + цель."""
    img = cv2.cvtColor(slam.map_array(), cv2.COLOR_GRAY2BGR)
    x, y, th = slam.pose_mm
    rc, rr = slam.world_to_pixel(x, y)
    # Стрелка курса: у Hector перёд = направление theta (col=x, row=y).
    L = 20
    hx = int(rc + L * math.cos(math.radians(th)))
    hy = int(rr + L * math.sin(math.radians(th)))
    cv2.arrowedLine(img, (rc, rr), (hx, hy), (0, 180, 0), 2, tipLength=0.45)
    cv2.circle(img, (rc, rr), 6, (0, 230, 0), -1)
    if navigator.path_px:
        for c, r in navigator.path_px:
            if 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
                img[r, c] = (255, 120, 0)
    if navigator.goal_px:
        cv2.drawMarker(img, navigator.goal_px, (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
    ok, buf = cv2.imencode(".png", img)
    return buf.tobytes() if ok else None


def run(cfg: FollowMeConfig) -> None:
    lidar = LidarThread(cfg.lidar_port, cfg.lidar_baud, cfg.lidar_max_range_m,
                        mount_offset_deg=cfg.lidar_mount_offset_deg)
    lidar.start()
    if not lidar.wait_until_ready(timeout=10.0):
        log.warning("LiDAR not ready yet -- continuing; distances appear once it spins up.")

    detector = Detector(cfg)
    visualizer = Visualizer(cfg)
    cap = open_camera(cfg)

    # IMU (BNO085) подключён напрямую к Jetson по I2C, читаем в фоне.
    imu = ImuReader(bus=cfg.imu_i2c_bus, address=cfg.imu_address)
    imu.start()

    # SLAM-бэкенд: hector (наш scan-to-map matching) или breezy (BreezySLAM).
    if cfg.slam_backend == "hector":
        from .hector import HectorSlam
        slam = HectorSlam(cfg)
        log.info("SLAM backend: Hector (scan-to-map matching)")
    else:
        slam = SlamMapper(cfg)
    navigator = Navigator(cfg, slam)
    last_slam = 0.0

    motor = (Esp32Motor(serial_port=cfg.motor_serial_port, baud=cfg.motor_baud)
             if cfg.enable_drive else None)
    if motor is not None:
        log.info("Drive ENABLED -> ESP32 serial %s (stop at %.2f m). "
                 "Движение только в режиме 'follow'.", cfg.motor_serial_port,
                 cfg.stop_distance_m)
    else:
        log.info("Drive disabled (perception only). Pass --drive to move the robot.")

    # Веб-интерфейс: при старте безопасный режим idle (моторы стоят), пока
    # оператор не выберет режим в браузере. follow двигает только при --drive.
    state = SharedState(mode="idle")
    web = None
    if cfg.enable_web:
        web = WebServer(state, port=cfg.web_port)
        web.start()

    # Resizable, movable window so it doesn't cover the whole desktop/terminal.
    if cfg.show_window:
        cv2.namedWindow(cfg.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(cfg.window_name, 1280, 480)

    fps = 0.0
    last = time.monotonic()
    sent_l = sent_r = 0  # фактически отправленные на моторы значения (для рампы)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                log.warning("Frame grab failed")
                continue

            detections = detector.detect(frame)
            tracks = fuse(detections, lidar, cfg)
            target = select_target(tracks)

            mode = state.mode
            now0 = time.monotonic()
            scan = lidar.get_scan()  # один раз за итерацию: для SLAM, веба, окна

            # Клик-цель с карты -> навигатору.
            goal_req = state.take_goal_request()
            if goal_req is not None:
                navigator.set_goal_px(*goal_req)

            # SLAM: поддерживаем карту+позу непрерывно (троттлинг под частоту лидара).
            if (slam.available() and lidar.wait_until_ready(timeout=0)
                    and (now0 - last_slam) >= cfg.slam_update_period_s):
                yv, yts = imu.get_yaw()
                yaw = yv if (yv is not None and now0 - yts < 1.0) else None
                # Средний ШИМ прошлой команды -> прайор смещения (одометрия без энкодеров).
                slam.update(scan, yaw,
                            (now0 - last_slam) if last_slam else cfg.slam_update_period_s,
                            fwd_pwm=(sent_l + sent_r) / 2.0)
                last_slam = now0
                if web is not None:
                    png = render_map_png(slam, navigator)
                    if png:
                        state.set_map_png(png)

            # Каждая ветка задаёт ЦЕЛЕВЫЕ (left, right); отправка моторам — единая,
            # с плавной рампой (см. ниже). Движение только в выбранном режиме.
            if mode == "follow":
                left, right, drive_status = compute_drive(target, cfg)
            elif mode == "mapping":
                # Ручной телеоп: команда с веб-пульта, протухает через deadman.
                tl, tr, tts = state.get_teleop()
                fresh = (time.monotonic() - tts) < cfg.teleop_deadman_s
                if (tl or tr) and fresh:
                    left, right = tl, tr
                    drive_status = f"карта/телеоп L={left} R={right}"
                else:
                    left, right = 0, 0
                    drive_status = "карта -- телеоп простаивает (удерживай кнопку)"
            elif mode == "goto":
                if not slam.available():
                    left, right = 0, 0
                    drive_status = "goto: SLAM недоступен (нет breezyslam)"
                else:
                    left, right, st = navigator.step(now0)
                    # Стоп по препятствию прямо по курсу (лидар 0deg = вперёд).
                    fwd = lidar.nearest_at(0.0, 40.0)
                    if (left > 0 or right > 0) and fwd is not None \
                            and fwd < cfg.nav_obstacle_stop_m:
                        left, right, st = 0, 0, f"препятствие спереди {fwd:.2f}м -> стоп"
                    drive_status = "goto: " + st
            else:
                left, right = 0, 0
                drive_status = "ожидание -- моторы остановлены"

            # Плавный разгон/торможение: тянем фактические команды к целевым (slew-rate).
            sent_l = _ramp(sent_l, left, cfg.motor_ramp_step)
            sent_r = _ramp(sent_r, right, cfg.motor_ramp_step)
            if motor is not None:
                if sent_l == 0 and sent_r == 0:
                    motor.stop()
                else:
                    motor.drive(sent_l, sent_r)

            if mode == "follow" and target is not None:
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

            # Публикуем размеченный кадр и состояние в веб-интерфейс.
            if web is not None:
                # Диагностика лидара: сколько точек в скане и ближайшая (для калибровки).
                valid = scan[~np.isnan(scan)]
                lidar_returns = int(valid.size)
                lidar_nearest = None
                if valid.size:
                    nidx = int(np.nanargmin(scan))
                    lidar_nearest = {"dist": round(float(scan[nidx]), 2), "angle": nidx}
                # Курс с IMU (BNO085 напрямую по I2C на Jetson), если свежий.
                imu_yaw = None
                y, yts = imu.get_yaw()
                if y is not None and (time.monotonic() - yts) < 1.0:
                    imu_yaw = round(y, 1)
                status = {
                    "mode": mode,
                    "mode_label": MODE_LABELS.get(mode, mode),
                    "fps": round(fps, 1),
                    "drive": motor is not None,
                    "left": left,
                    "right": right,
                    "drive_status": drive_status,
                    "lidar_ready": lidar.wait_until_ready(timeout=0),
                    "lidar_returns": lidar_returns,
                    "lidar_nearest": lidar_nearest,
                    "imu_yaw": imu_yaw,
                    "slam": slam.available(),
                    "pose": ({"x": round(slam.pose_mm[0] / 1000.0, 2),
                              "y": round(slam.pose_mm[1] / 1000.0, 2),
                              "theta": round(slam.pose_mm[2], 0)}
                             if slam.available() else None),
                    "robot_px": (list(slam.world_to_pixel(slam.pose_mm[0],
                                                          slam.pose_mm[1]))
                                 if slam.available() else None),
                    "map_px": cfg.map_size_pixels,
                    "n_people": len(tracks),
                    "target": None if target is None else {
                        "angle": round(target.cam_angle_deg, 1),
                        "dist": (None if target.distance_m is None
                                 else round(target.distance_m, 2)),
                    },
                }
                jpeg = encode_jpeg(annotate(frame.copy(), tracks, target, status))
                # Скан для калибровки: [угол, дистанция] по всем возвратам.
                scan_points = [[int(a), round(float(scan[a]), 2)]
                               for a in np.where(~np.isnan(scan))[0]]
                state.publish(jpeg, status, scan_points)

            if cfg.show_window:
                canvas = visualizer.render(frame, tracks, scan, target, fps,
                                           cmd=(left, right, drive_status))
                cv2.imshow(cfg.window_name, canvas)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        if motor is not None:
            motor.stop()      # leave the robot stopped on exit
            motor.close()
        if web is not None:
            web.stop()
        imu.stop()
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
    p.add_argument("--motor-port", dest="motor_serial_port",
                   default=cfg.motor_serial_port, help="ESP32 USB-serial порт")
    p.add_argument("--no-web", dest="enable_web", action="store_false",
                   default=cfg.enable_web, help="не запускать веб-интерфейс")
    p.add_argument("--web-port", dest="web_port", type=int, default=cfg.web_port)
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
    # rplidar спамит INFO "Asking for health" каждый скан -- глушим до WARNING.
    logging.getLogger("rplidar").setLevel(logging.WARNING)
    run(parse_args())


if __name__ == "__main__":
    main()
