#!/usr/bin/env bash
# Convenience launcher on the Jetson. The OpenCV window appears on the monitor
# physically attached to the Jetson (DISPLAY=:0), even when started over SSH.
#
#   bash scripts/run.sh                 # default config
#   bash scripts/run.sh --hfov 78       # pass-through args to follow_me.main

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# Activate venv if present.
if [ -d .venv ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

# Target the locally-attached display so cv2.imshow renders on the Jetson monitor.
export DISPLAY="${DISPLAY:-:0}"
# Allow the SSH session's user to draw on the local X server if needed.
xhost +SI:localuser:"$(whoami)" >/dev/null 2>&1 || true

exec python -m follow_me.main "$@"
