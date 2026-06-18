#!/usr/bin/env bash
# One-time environment setup on the Jetson. Run via SSH after deploy.sh:
#   cd ~/greenhouse && bash scripts/jetson_setup.sh
#
# Creates a venv that REUSES the JetPack system packages (OpenCV + TensorRT),
# installs the GPU PyTorch wheels matching this JetPack, then the Python deps.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."   # project root

echo "================ 1. Detect JetPack / L4T ================"
L4T_LINE="$(cat /etc/nv_tegra_release 2>/dev/null || true)"
echo "${L4T_LINE:-/etc/nv_tegra_release not found}"
# R36.x => JetPack 6 (Ubuntu 22.04, py3.10); R35.x => JetPack 5 (Ubuntu 20.04, py3.8)
if echo "$L4T_LINE" | grep -q 'R36'; then
  JP="jp6"
elif echo "$L4T_LINE" | grep -q 'R35'; then
  JP="jp5"
else
  JP="unknown"
fi
PY="$(python3 -V 2>&1)"
echo "Detected: JetPack=${JP}  ${PY}"
if [ "$JP" = "unknown" ]; then
  echo "!! Could not auto-detect JetPack. Inspect the R3x.x value above and pick the"
  echo "!! matching torch wheel from https://pypi.jetson-ai-lab.dev/ manually."
fi

echo "================ 2. Power mode (max GPU) ================"
sudo nvpmodel -m 0 || echo "(nvpmodel failed/!available -- skip)"
sudo jetson_clocks || echo "(jetson_clocks failed -- skip)"

echo "================ 3. Serial access to LiDAR ================"
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "Added $USER to 'dialout'. LOG OUT/IN (or reboot) for it to take effect."
fi

echo "================ 4. APT prerequisites ================"
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip libopenblas-base libopenmpi-dev

echo "================ 5. Python venv (system packages = cv2 + tensorrt) ================"
if [ ! -d .venv ]; then
  python3 -m venv --system-site-packages .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip

echo "================ 6. GPU PyTorch wheels (Jetson) ================"
# IMPORTANT: index host is jetson-ai-lab.IO (not .dev). Generic PyPI torch does NOT
# work on Jetson (it can't see the Tegra GPU), so we pin Jetson-built wheels and
# install them with --no-deps over PyPI-provided dependencies.
# cuSPARSELt is a runtime dep of the Jetson torch build.
sudo apt-get install -y libcusparselt0 libcusparselt-dev || true
# Remove any stray generic torch from the user site (it would shadow the venv).
rm -rf "$HOME/.local/lib/python3.10/site-packages/torch" \
       "$HOME/.local/lib/python3.10/site-packages/torchvision" 2>/dev/null || true
pip uninstall -y torch torchvision torchaudio 2>/dev/null || true
pip install --no-cache-dir "numpy<2" pillow filelock typing-extensions sympy networkx jinja2 fsspec
if [ "$JP" = "jp6" ]; then
  pip install --no-cache-dir --no-deps torch==2.8.0 torchvision==0.23.0 \
    --index-url=https://pypi.jetson-ai-lab.io/jp6/cu126 || \
    echo "!! torch install failed -- pick versions for your JetPack at https://pypi.jetson-ai-lab.io/jp6/cu126"
elif [ "$JP" = "jp5" ]; then
  pip install --no-cache-dir --no-deps torch torchvision \
    --index-url=https://pypi.jetson-ai-lab.io/jp5/cu114 || \
    echo "!! torch install failed -- see https://pypi.jetson-ai-lab.io/jp5/"
else
  echo "!! Skipping torch -- install manually for your JetPack, then re-run from step 7."
fi

echo "================ 7. Project Python deps ================"
# torch/torchvision already installed above; cv2 + tensorrt come from system packages.
pip install --no-cache-dir ultralytics rplidar-roboticia pyserial
# ONNX export stack needed to build the TensorRT engine. onnxruntime-gpu has no PyPI
# aarch64 wheel -- pull it from the Jetson index; fall back to CPU onnxruntime.
pip install --no-cache-dir onnx onnxslim
if [ "$JP" = "jp6" ]; then
  pip install --no-cache-dir --no-deps onnxruntime-gpu \
    --index-url=https://pypi.jetson-ai-lab.io/jp6/cu126 || \
    pip install --no-cache-dir onnxruntime || true
fi

echo "================ 8. Sanity check ================"
python - <<'PY'
import importlib
for m in ("cv2", "torch", "ultralytics", "rplidar", "serial"):
    try:
        mod = importlib.import_module(m)
        v = getattr(mod, "__version__", "?")
        print(f"  OK  {m} {v}")
    except Exception as e:
        print(f"  !!  {m}: {e}")
try:
    import torch
    print(f"  torch.cuda.is_available() = {torch.cuda.is_available()}")
except Exception as e:
    print(f"  !! torch cuda check: {e}")
PY

echo
echo "Setup done. Next:"
echo "  source .venv/bin/activate"
echo "  python scripts/check_devices.py"
echo "  python scripts/export_yolo_tensorrt.py     # builds models/yolo11n.engine"
echo "  DISPLAY=:0 python -m follow_me.main         # window shows on the Jetson monitor"
