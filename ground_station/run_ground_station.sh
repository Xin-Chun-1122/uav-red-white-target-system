#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORIN_IP="${ORIN_IP:-10.0.0.8}"
PORT="${PORT:-5001}"
FULLSCREEN="${FULLSCREEN:-0}"
QT_PLUGIN_DIR="${QT_PLUGIN_DIR:-/usr/lib/aarch64-linux-gnu/qt5/plugins}"

# OpenCV wheels ship their own Qt plugin folder, which can shadow PyQt5's xcb
# plugin and make the GUI exit immediately when started from this script.
export QT_QPA_PLATFORM_PLUGIN_PATH="$QT_PLUGIN_DIR"
unset QT_PLUGIN_PATH

args=(python3 "$SCRIPT_DIR/red_white_ground_gui.py" --orin-ip "$ORIN_IP" --port "$PORT")

echo "Starting mission-target ground GUI"
echo "  Orin: http://$ORIN_IP:$PORT"
echo "  DISPLAY: ${DISPLAY:-not set}"
echo "  FULLSCREEN: $FULLSCREEN"
echo

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set. Run this on the laptop desktop terminal, not a headless SSH session." >&2
  exit 2
fi

if [[ "$FULLSCREEN" == "1" ]]; then
  args+=(--fullscreen)
fi

exec "${args[@]}"
