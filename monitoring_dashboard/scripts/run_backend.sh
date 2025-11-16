#!/bin/bash

# Monitoring Dashboard Backend Startup Script
# Starts FastAPI server with WebSocket support

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/../backend"

echo "================================================"
echo "  Multi-Agent Formation Control Monitor"
echo "  Backend Server (FastAPI + ROS2 Bridge)"
echo "================================================"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 environment not sourced!"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Check if workspace is sourced
if [ -z "$COLCON_PREFIX_PATH" ]; then
    echo "WARNING: Workspace not sourced. Attempting to source..."
    WORKSPACE_ROOT="$SCRIPT_DIR/../../.."
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
        echo "✓ Workspace sourced successfully"
    else
        echo "ERROR: Could not find workspace setup file"
        exit 1
    fi
fi

# Check Python dependencies
echo "Checking Python dependencies..."
python3 -c "import fastapi" 2>/dev/null || {
    echo "ERROR: fastapi not installed"
    echo "Install with: pip3 install fastapi uvicorn websockets pydantic"
    exit 1
}

python3 -c "import rclpy" 2>/dev/null || {
    echo "ERROR: rclpy not installed"
    echo "Make sure ROS2 Python bindings are available"
    exit 1
}

echo "✓ All dependencies OK"
echo ""

# Navigate to backend directory
cd "$BACKEND_DIR"

# Start backend server
echo "Starting backend server on http://localhost:8000"
echo "WebSocket endpoint: ws://localhost:8000/ws"
if [ -d "$SCRIPT_DIR/../frontend/dist" ]; then
    echo "Frontend UI available at: http://localhost:8000/ui"
else
    echo "(Tip) Build frontend to serve UI from backend:"
    echo "     cd monitoring_dashboard/frontend && npm run build"
    echo "     then open http://localhost:8000/ui"
fi
echo "Press Ctrl+C to stop"
echo ""

python3 app.py
