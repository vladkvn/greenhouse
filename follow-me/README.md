# Follow-Me Perception (Jetson Orin Nano 8GB)

Perception module for a follow-me robot. On an NVIDIA Jetson Orin Nano with a USB HD
camera and an RPLiDAR A1M8 it:

- detects people on the camera feed **on the GPU** (Ultralytics YOLO → TensorRT, FP16),
- visualizes the LiDAR as a top-down plot,
- fuses both to draw, for each person, the **direction** (bearing) and **distance**.

The selected target's `(angle, distance)` is logged every frame as the integration point
for future motion control.

```
follow_me/
  config.py      # all tunables (ports, HFOV, alignment, thresholds)
  lidar.py       # background RPLiDAR reader -> 360-bin scan
  detector.py    # YOLO person detector (TensorRT engine, .pt fallback)
  fusion.py      # bbox -> camera bearing -> LiDAR angle -> distance
  visualizer.py  # OpenCV: camera panel | top-down LiDAR panel
  main.py        # pipeline + graceful shutdown
scripts/
  export_yolo_tensorrt.py  # build the TensorRT engine (run on the Jetson)
  check_devices.py         # verify camera + LiDAR
tests/                     # pure-logic tests (run anywhere, no hardware)
```

## Setup on the Jetson

1. **Power/clocks** for full GPU throughput:
   ```bash
   sudo nvpmodel -m 0      # MAXN
   sudo jetson_clocks
   ```
2. **Serial access** to the LiDAR (CP2102 on `/dev/ttyUSB0`), then re-login:
   ```bash
   sudo usermod -aG dialout $USER
   ```
3. **PyTorch/torchvision**: install the NVIDIA Jetson wheels matching your JetPack
   *before* the other deps (see the note in `requirements.txt`). TensorRT comes with
   JetPack — do not pip-install it.
4. **Python deps**:
   ```bash
   pip install -r requirements.txt
   ```

## Run

```bash
# 1. Check the hardware is visible
python scripts/check_devices.py

# 2. Build the TensorRT engine ON THE JETSON (one-time, ~minutes)
python scripts/export_yolo_tensorrt.py            # -> models/yolo11n.engine

# 3. Run perception
python -m follow_me.main
#   --hfov 70 --offset 0 --camera-index 0 --lidar-port /dev/ttyUSB0 --no-window
```

Press `q` to quit. Left panel: people in boxes with distance/bearing and a direction
arrow. Right panel: top-down LiDAR with range rings; the target's bearing is highlighted.

## Calibration

Camera and LiDAR are assumed coaxial and forward-facing. To calibrate:

- **Distance/bearing match**: stand centred in the frame at a known distance (e.g. 2.0 m).
  The on-screen value should match. If a centred person reads the wrong distance, the
  LiDAR zero is misaligned — adjust `cam_to_lidar_offset_deg` (`--offset`).
- **Left/right swapped**: set `lidar_flip = True` (`--flip`).
- **Lens FOV**: set `camera_hfov_deg` (`--hfov`) to your lens's true horizontal FOV.

## Tests

Pure geometry and LiDAR-binning logic run without any hardware or the `rplidar`/`torch`
packages:

```bash
pip install pytest numpy
pytest -q
```

## Verify GPU usage

While `follow_me.main` runs, `jtop` (from `jetson-stats`) should show GPU activity and the
window should report ~25–30+ FPS with the TensorRT engine.

## Out of scope (next steps)

Motor control / ROS2 / per-ID tracking (ByteTrack) for robust following. The steering
command `(target_angle, target_dist)` is already computed and logged here.

## Motor control (ESP32) — follow for real

Perception now drives the robot. The selected target `(bearing, distance)` is turned
into skid-steer commands and sent to the ESP32 motor module over WiFi/UDP
(`follow_me/motor.py`, protocol `"L R"` / `"STOP"`, port 4210).

```bash
cd ~/greenhouse && source .venv/bin/activate

# perception only — computes & logs the command but does NOT move (default, safe):
python -m follow_me.main --no-window

# drive for real — sends commands to the ESP32 (PROP UP THE WHEELS FIRST):
python -m follow_me.main --no-window --drive
#   --esp32 192.168.1.50   ESP32 IP (default)
```

Stops when the followed person is within `stop_distance_m` (1.0 m, LiDAR-gated); eases
off below `slow_distance_m`. Tune speeds in `follow_me/config.py` (`motor_*`). The ESP32
has its own 0.5 s failsafe: if the script dies, the motors stop on their own.

### ESP32 firmware

The motor module firmware is in [`firmware_esp32/`](firmware_esp32/firmware_esp32.ino):
plain ESP32, static IP `192.168.1.50`, UDP `4210`, `"L R"`/`"STOP"` → 4 motors via one
L298 (left pair = channel A, right pair = channel B), 0.5 s command-watchdog failsafe.
Flash with `arduino-cli` (esp32 core 3.x); on CH340 boards upload at `UploadSpeed=115200`.
Pinout and wiring notes are in the sketch header.
