#!/usr/bin/env bash
# Deploy the follow_me project to the Jetson over SSH with rsync.
#
# Usage:
#   JETSON_HOST=user@jetson.local ./scripts/deploy.sh
#   JETSON_HOST=nvidia@192.168.1.50 JETSON_DIR=~/greenhouse ./scripts/deploy.sh
#
# Re-run any time you change code locally; rsync only sends the diff.

set -euo pipefail

: "${JETSON_HOST:?Set JETSON_HOST, e.g. JETSON_HOST=nvidia@192.168.1.50}"
# Relative path => interpreted against the Jetson's home dir. Do NOT use a leading
# "~" here: your local shell would expand it to the Mac home before rsync runs.
JETSON_DIR="${JETSON_DIR:-greenhouse}"
case "$JETSON_DIR" in
  /Users/*|"$HOME"/*)
    echo "!! JETSON_DIR='$JETSON_DIR' looks like a Mac path (tilde expanded locally)."
    echo "!! Run:  unset JETSON_DIR   (uses 'greenhouse' on the Jetson home), then retry."
    exit 1 ;;
esac

# Project root = parent of this script's directory.
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/"

echo ">> ensuring ${JETSON_HOST}:${JETSON_DIR} exists"
ssh "${JETSON_HOST}" "mkdir -p \"${JETSON_DIR}\""

echo ">> rsync ${SRC}  ->  ${JETSON_HOST}:${JETSON_DIR}"
rsync -avz --delete \
  --exclude '.git/' \
  --exclude '__pycache__/' \
  --exclude '.pytest_cache/' \
  --exclude '*.pyc' \
  --exclude '.venv/' \
  --exclude 'models/*.engine' \
  "${SRC}" "${JETSON_HOST}:${JETSON_DIR}/"

echo ">> Done. Next on the Jetson:"
echo "   ssh ${JETSON_HOST}"
echo "   cd ${JETSON_DIR} && bash scripts/jetson_setup.sh"
