#!/bin/bash
set -e

# Source ROS 2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble setup file not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Configure FastDDS to use UDP only (avoid SHM errors)
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastdds_profiles.xml

echo "Cleaning build artifacts..."
rm -rf build install log

echo "Building workspace..."
colcon build --symlink-install

echo "Build complete. Don't forget to source the setup file:"
echo "source install/setup.bash"
