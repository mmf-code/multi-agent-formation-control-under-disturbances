#!/bin/bash

# Setup script for monitoring dashboard
# Installs all required dependencies

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."

echo "================================================"
echo "  Monitoring Dashboard Setup"
echo "================================================"
echo ""

# Check system
echo "Checking system requirements..."

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 not found"
    exit 1
fi

# Check Node.js
if ! command -v node &> /dev/null; then
    echo "ERROR: Node.js not found"
    echo "Install with: sudo apt install nodejs npm"
    exit 1
fi

# Check ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "WARNING: ROS2 environment not sourced"
    echo "Please source ROS2 before running the backend"
fi

echo "✓ System requirements OK"
echo ""

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --user fastapi uvicorn websockets pydantic

echo "✓ Python dependencies installed"
echo ""

# Install Node.js dependencies
echo "Installing Node.js dependencies..."
cd "$PROJECT_ROOT/frontend"
npm install

echo "✓ Node.js dependencies installed"
echo ""

# Make scripts executable
echo "Setting executable permissions..."
chmod +x "$SCRIPT_DIR/run_backend.sh"
chmod +x "$SCRIPT_DIR/run_frontend.sh"

echo "✓ Setup complete!"
echo ""
echo "================================================"
echo "  Next Steps:"
echo "================================================"
echo ""
echo "1. Source your ROS2 workspace:"
echo "   source /opt/ros/<distro>/setup.bash"
echo "   source <workspace>/install/setup.bash"
echo ""
echo "2. Start the simulation (in a new terminal)"
echo ""
echo "3. Start the backend server:"
echo "   ./scripts/run_backend.sh"
echo ""
echo "4. Start the frontend (in a new terminal):"
echo "   ./scripts/run_frontend.sh"
echo ""
echo "5. Open browser to http://localhost:3000"
echo ""
