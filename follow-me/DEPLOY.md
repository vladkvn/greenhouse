# Deploy & run on the Jetson Orin Nano

Deploy from your Mac over SSH with `rsync`, set up the environment once on the Jetson,
then run. The OpenCV window appears on the **monitor attached to the Jetson**.

Prereqs on the Mac: SSH access to the Jetson, and `rsync` (preinstalled on macOS).
Find the Jetson address with `hostname -I` on the device, or use its `.local` name.

---

## 1. Deploy the code (from the Mac)

```bash
cd ~/work/greenhouse

# Set your Jetson SSH target once per shell:
export JETSON_HOST=luki@192.168.1.58     # or e.g. nvidia@jetson.local
# JETSON_DIR is OPTIONAL and defaults to 'greenhouse' (relative to the Jetson home).
# Do NOT export it as ~/greenhouse — the tilde expands on the Mac and breaks rsync.
# If you must set it, use a plain relative path:  export JETSON_DIR=greenhouse

./scripts/deploy.sh
```

`deploy.sh` rsyncs the project (excluding `.git`, caches, `.venv`, and the local
`*.engine`) to `~/greenhouse` on the Jetson. Re-run it any time you change code — only
the diff is sent.

---

## 2. One-time setup (on the Jetson)

```bash
ssh "$JETSON_HOST"
cd ~/greenhouse
bash scripts/jetson_setup.sh
```

This script:

1. **Detects JetPack** from `/etc/nv_tegra_release` (R36→JP6 / R35→JP5).
2. Sets **max power** (`nvpmodel -m 0` + `jetson_clocks`).
3. Adds you to **`dialout`** for `/dev/ttyUSB0` (LiDAR) — **log out/in or reboot** after.
4. Installs apt prereqs.
5. Creates a venv with **`--system-site-packages`** so it reuses JetPack's **OpenCV and
   TensorRT** (do not pip-install those — the system builds are CUDA-enabled).
6. Installs **GPU PyTorch/torchvision** from the Jetson wheel index
   (`pypi.jetson-ai-lab.dev`) matching the detected JetPack.
7. Installs `ultralytics`, `rplidar-roboticia`, `pyserial`.
8. Prints a sanity check, including `torch.cuda.is_available()`.

> If JetPack is `unknown` or the torch install fails, open
> <https://pypi.jetson-ai-lab.dev/> , pick the index URL for your `R3x.x` / CUDA, edit
> step 6 of `scripts/jetson_setup.sh`, and re-run it. Everything is idempotent.

After it finishes (and after a re-login for the `dialout` group):

```bash
source ~/greenhouse/.venv/bin/activate
```

---

## 3. Verify hardware (on the Jetson)

```bash
cd ~/greenhouse && source .venv/bin/activate
python scripts/check_devices.py
```

Expect `camera=OK  lidar=OK`. If the LiDAR fails with a permission error, the `dialout`
group membership hasn't taken effect yet — re-login/reboot.

---

## 4. Build the TensorRT engine (on the Jetson, one-time, ~minutes)

The engine is GPU/TensorRT-specific, so it **must** be built on the device (it is not
rsynced from the Mac).

```bash
python scripts/export_yolo_tensorrt.py        # downloads yolo11n.pt, writes models/yolo11n.engine
```

---

## 5. Run

Over SSH, the window targets the Jetson's local monitor via `DISPLAY=:0`:

```bash
bash scripts/run.sh
# pass-through args, e.g. calibration:
bash scripts/run.sh --hfov 78 --offset 0
```

Left panel: people in boxes with distance + bearing and a direction arrow.
Right panel: top-down LiDAR with range rings; the target's bearing highlighted.
Console logs `STEER angle=… dist=…` each frame. Press **q** in the window to quit.

Headless fallback (no monitor / just logs):

```bash
python -m follow_me.main --no-window
```

---

## 6. Verify GPU usage

In a second SSH session:

```bash
sudo pip install -U jetson-stats   # once
jtop                               # GPU tab should show load while main.py runs
```

Target throughput with the TensorRT engine: ~25–30+ FPS (shown top-left in the window).

---

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `cv2`/`tensorrt` missing in venv | venv must be created with `--system-site-packages` (re-run setup) |
| `torch.cuda.is_available() = False` | generic PyPI torch got installed — remove it (incl. `~/.local/.../torch*`) and install Jetson wheels: `pip install --no-deps torch==2.8.0 torchvision==0.23.0 --index-url=https://pypi.jetson-ai-lab.io/jp6/cu126` |
| `libcusparseLt.so.0: cannot open shared object file` | `sudo apt-get install -y libcusparselt0 libcusparselt-dev` |
| LiDAR `permission denied` on `/dev/ttyUSB0` | re-login after `usermod -aG dialout`; check `ls -l /dev/ttyUSB0` |
| Window doesn't appear over SSH | use `scripts/run.sh` (sets `DISPLAY=:0`); ensure a desktop session is active on the Jetson |
| Wrong camera | `--camera-index 1` (list with `ls /dev/video*`) |
| Distance/bearing off | calibrate `--hfov`, `--offset`, `--flip` (see README “Calibration”) |
| Low FPS / using `.pt` | the `.engine` is missing — run step 4; confirm MAXN + `jetson_clocks` |
