#!/bin/bash
# Small helper to publish wind to Gazebo/ROS topics using wind_tool.py
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Defaults (can be overridden via args)
X=2.0
Y=8.0
Z=0.0
DURATION=60
MODE="both"   # force, velocity, or both
RATE=10.0
SCALE=1.0

usage() {
  echo "Usage: $0 [--x <val>] [--y <val>] [--z <val>] [--duration <sec>] [--mode force|velocity|both] [--rate <hz>] [--scale <factor>]"
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --x) X="$2"; shift 2 ;;
    --y) Y="$2"; shift 2 ;;
    --z) Z="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --scale) SCALE="$2"; shift 2 ;;
    --help|-h) usage ;;
    *)
      echo "Unknown argument: $1"
      usage
      ;;
  esac
done

# Source ROS environments so rclpy/msgs are available
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

WIND_TOOL="${SCRIPT_DIR}/wind_tool.py"
if [ ! -f "${WIND_TOOL}" ]; then
  echo "wind_tool.py not found at ${WIND_TOOL}"
  exit 1
fi

echo "[wind] Publishing wind (mode=${MODE}, scale=${SCALE}) for ${DURATION}s: x=${X}, y=${Y}, z=${Z}"
python3 "${WIND_TOOL}" \
  --x "${X}" \
  --y "${Y}" \
  --z "${Z}" \
  --duration "${DURATION}" \
  --mode "${MODE}" \
  --rate "${RATE}" \
  --scale "${SCALE}"
