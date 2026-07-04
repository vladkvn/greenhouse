"""Configuration for the follow-me perception module.

All tunables live here. The defaults assume a Jetson Orin Nano with a USB HD camera
and an RPLiDAR A1M8 mounted coaxially (both facing forward).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FollowMeConfig:
    # --- Camera ---
    camera_index: int = 0
    frame_w: int = 1280
    frame_h: int = 720
    # Horizontal field of view of the camera lens, degrees. Calibrate per lens.
    camera_hfov_deg: float = 70.0

    # --- LiDAR (RPLiDAR A1M8) ---
    lidar_port: str = "/dev/ttyUSB0"
    lidar_baud: int = 115200
    # RPLiDAR A1 sees up to ~12 m; clamp anything beyond as "no return".
    lidar_max_range_m: float = 12.0

    # --- Camera <-> LiDAR alignment ---
    # Offset added to the camera-derived angle to land on the LiDAR's 0deg=forward axis.
    # 0.0 for coaxial forward mounting; calibrate empirically otherwise.
    cam_to_lidar_offset_deg: float = 0.0
    # If the LiDAR spins so that "right" maps to negative angles, flip the sign.
    lidar_flip: bool = False
    # Угол на лидаре, соответствующий "перёд робота". Скан разворачивается в источнике на
    # этот угол, чтобы бин 0 = перёд (фьюжн/стоп/карта). Лидар развёрнут ~180° -> 180.
    lidar_mount_offset_deg: int = 180
    # Angular window (total width) around the person's angle to sample LiDAR returns.
    # Wider than the person's angular size so camera<->LiDAR parallax at range still
    # catches them; fuse() takes the NEAREST return in this window (the person, not
    # the wall behind). See LidarThread.nearest_at.
    fusion_window_deg: float = 8.0

    # --- Detector (Ultralytics YOLO -> TensorRT) ---
    engine_path: str = "models/yolo11n.engine"
    # Fallback weights if the TensorRT engine is missing (still runs on GPU, slower).
    fallback_weights: str = "yolo11n.pt"
    conf_thr: float = 0.45
    person_class_id: int = 0
    infer_imgsz: int = 640
    device: int | str = 0  # CUDA device index for inference

    # --- Motor control (ESP32 over USB-serial) ---
    # ESP32 подключён по USB (CH340). Протокол "L R\n" / "STOP\n" @115200.
    motor_serial_port: str = "/dev/ttyUSB0"
    motor_baud: int = 115200
    # Оставлены для совместимости со старым UDP-вызовом Esp32Motor(ip, port); не используются.
    esp32_ip: str = "192.168.1.50"
    esp32_port: int = 4210
    # Motion is OFF by default; enable with --drive so running perception never moves
    # the robot unexpectedly.
    enable_drive: bool = False
    # Stop when the followed person is at/under this range (m). LiDAR-gated.
    stop_distance_m: float = 1.0
    # Start easing off the throttle below this range (m).
    slow_distance_m: float = 1.6
    # PWM magnitudes (0..255). Open-loop -- tune for your chassis/battery.
    motor_base_speed: int = 170   # cruise forward
    motor_max_speed: int = 220    # per-side ceiling
    motor_min_move: int = 120     # L298 deadzone: below this a motor won't spin
    motor_turn_gain: float = 1.3  # steering authority (x base_speed at full bearing)
    # Плавный старт/стоп: макс. изменение ШИМ за тик управления (slew-rate).
    motor_ramp_step: int = 20

    # --- Visualization ---
    lidar_panel_size: int = 720  # square LiDAR panel side, px
    window_name: str = "follow-me perception"
    show_window: bool = True

    # --- IMU (BNO085 напрямую на I2C Jetson, 40-пиновый гребень pins 3/5) ---
    imu_i2c_bus: int = 7      # /dev/i2c-7
    imu_address: int = 0x4A

    # --- Веб-интерфейс ---
    enable_web: bool = True
    web_port: int = 8080
    # Телеоп: команда с веб-пульта протухает через столько секунд -> STOP (deadman).
    teleop_deadman_s: float = 0.6

    # --- SLAM ---
    # Бэкенд: "hector" (scan-to-map matching, Gauss-Newton, наш) или "breezy" (BreezySLAM).
    slam_backend: str = "hector"
    # Квадратная карта: сторона в пикселях и в метрах (10 м / 500 px = 2 см/пиксель).
    map_size_pixels: int = 500
    map_size_meters: float = 10.0
    # --- Hector SLAM (наш scan matcher) ---
    hector_iters: int = 6            # итераций Gauss-Newton на уровень
    hector_scales: tuple = (4, 2, 1)  # multi-res: грубо->точно (max-pool, шире захват)
    hector_scan_dir: int = 1         # направление обхода лидара (+1/-1), калибровка
    hector_occ_gain: float = 0.5     # сила пометки занятой ячейки (к 1)
    hector_free_gain: float = 0.08   # сила пометки свободной (к 0)
    hector_warmup_scans: int = 4     # сначала строим карту, потом матчим
    hector_max_jump_m: float = 0.6   # скачок позы больше -> расхождение, отброс
    hector_max_jump_deg: float = 35.0
    # Прайор движения для matcher (ключево для поворотов): IMU поворот + ШИМ смещение.
    hector_use_imu: bool = True
    hector_imu_sign: int = -1        # знак dtheta IMU в кадр Hector (калибровка)
    hector_use_odom: bool = True
    # Курс: False -> берём с IMU (scan-matching правит только x,y; устойчиво у колонн,
    # где угол неоднозначен). True -> GN правит и theta (тест математики гоняет True).
    hector_match_theta: bool = False
    # Параметры лидара для модели Laser (RPLIDAR A1): точек/скан, Гц, угол обзора, макс. мм.
    slam_scan_size: int = 360
    slam_scan_rate_hz: float = 5.5
    slam_detection_deg: float = 360.0
    slam_no_detection_mm: int = 12000
    # Обновлять SLAM не чаще этого периода (лидар ~5.5 Гц, чаще нет смысла).
    slam_update_period_s: float = 0.18
    # Псевдо-одометрия: оценка скорости робота при ШИМ 255 (м/с). Даёт SLAM прайор
    # смещения (энкодеров нет). Калибруется: проедь 2 м, сверь путь робота на карте.
    odom_speed_full_mps: float = 0.35
    # Внутренний "перёд" BreezySLAM (theta) развёрнут на 180° от реального переда робота
    # (замер: bearing движения = slam_theta+180). Поэтому dxy одометрии инвертируем,
    # иначе SLAM толкает позу НАЗАД при езде вперёд -> карта плывёт (особенно в goto).
    slam_odom_invert: bool = True
    # Инверсия знака поворота для SLAM, если карта крутится не туда (калибровка).
    slam_dtheta_invert: bool = True
    # Параметры BreezySLAM RMHC. hole_width -- толщина рисуемой стены (мм): дефолт 600
    # (60см!) даёт «штрихи»; 200 -> тонкие стены. quality -- сила впечатывания скана.
    # sigma_xy/theta -- окно поиска совмещения (мм/град) вокруг прайора одометрии.
    slam_hole_width_mm: int = 200
    slam_map_quality: int = 50
    slam_sigma_xy_mm: int = 100
    slam_sigma_theta_deg: int = 20
    slam_max_search_iter: int = 1500

    # --- Навигация (езда в точку) ---
    # Радиус робота + желаемый зазор до стен -> раздув препятствий в планировщике (м).
    robot_radius_m: float = 0.15
    nav_wall_clearance_m: float = 0.15   # не подъезжать к стенам ближе этого
    # Сдвиг курса навигации: перёд робота = SLAM theta + это. Hector: 0 (перёд=theta).
    nav_heading_offset_deg: float = 0.0
    # Считаем точку достигнутой в пределах этого радиуса (м).
    goal_tolerance_m: float = 0.25
    # Дистанция упреждения pure-pursuit (м).
    lookahead_m: float = 0.6
    # Реактивный стоп: препятствие по лидару ближе этого по курсу движения (м).
    nav_obstacle_stop_m: float = 0.22
    # Планирование A* на загрублённой в N раз сетке -> быстрее (не роняет FPS).
    nav_plan_scale: int = 4
    # Базовая и поворотная скорость навигации (ШИМ 0..255). Выше порога срыва (~120).
    nav_speed: int = 140
    nav_turn_gain: float = 1.1
    # Инверсия поворота: карта y-вниз (зеркальная) -> физический поворот обратный.
    nav_turn_invert: bool = False
    # Мёртвая зона по курсу: |ошибка|<этого -> не доворачиваем (гасит маятник), град.
    nav_heading_deadband_deg: float = 8.0
    # Выше arc -> едем медленно с доворотом (дуга); выше spin -> разворот на месте, град.
    nav_arc_deg: float = 50.0
    nav_spin_deg: float = 110.0
    # Замедление при подъезде к цели ближе этого радиуса (м).
    nav_slow_radius_m: float = 0.7

    # --- LiDAR 2D map (clustering of returns into objects/walls) ---
    # Start a new object when consecutive returns jump more than this in angle or range.
    map_cluster_gap_deg: float = 6.0
    map_cluster_jump_m: float = 0.35
    # Clusters with at least this many points are drawn as filled object blobs.
    map_min_cluster_pts: int = 3
    # A person's LiDAR angle within this tolerance of a cluster marks it as the person.
    map_person_match_m: float = 0.6

    @property
    def fusion_half_window_deg(self) -> float:
        return self.fusion_window_deg / 2.0
