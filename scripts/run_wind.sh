#!/bin/bash

# Convenience launcher for constant wind in simulation
# Usage example:
#   ./scripts/run_wind.sh --x 4.0 --y 1.2 --z 0.0 --duration 60

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCRIPT_DIR}/.."

cd "${WS_ROOT}"

if [ -z "${ROS_DISTRO}" ]; then
  # Source ROS 2 (assume Humble)
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/opt/ros/humble/setup.bash"
  else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
  fi
fi

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
fi

exec python3 scripts/wind_tool.py "$@"

