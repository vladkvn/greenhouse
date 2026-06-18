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
    # Angular window (total width) around the person's angle to sample LiDAR returns.
    fusion_window_deg: float = 4.0

    # --- Detector (Ultralytics YOLO -> TensorRT) ---
    engine_path: str = "models/yolo11n.engine"
    # Fallback weights if the TensorRT engine is missing (still runs on GPU, slower).
    fallback_weights: str = "yolo11n.pt"
    conf_thr: float = 0.45
    person_class_id: int = 0
    infer_imgsz: int = 640
    device: int | str = 0  # CUDA device index for inference

    # --- Motor control (ESP32 over WiFi/UDP) ---
    # The ESP32 motor module has a static IP and speaks "L R" / "STOP" on this port.
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

    # --- Visualization ---
    lidar_panel_size: int = 720  # square LiDAR panel side, px
    window_name: str = "follow-me perception"
    show_window: bool = True

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
