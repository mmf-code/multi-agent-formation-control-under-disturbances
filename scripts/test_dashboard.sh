#!/bin/bash
################################################################################
# Dashboard Test Script
# Tests that dashboard can receive ROS topics and display data correctly
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║       Dashboard Integration Test                      ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Workspace
WORKSPACE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
cd "$WORKSPACE_DIR"

# Setup ROS2
echo -e "${CYAN}[1/5] Setting up ROS2...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start backend
echo -e "${CYAN}[2/5] Starting dashboard backend...${NC}"
cd monitoring_dashboard/backend
python3 app.py &
BACKEND_PID=$!
echo -e "  → Backend PID: ${BACKEND_PID}"
cd ../..

# Wait for backend
sleep 3

# Start frontend dev server
echo -e "${CYAN}[3/5] Starting frontend dev server...${NC}"
cd monitoring_dashboard/frontend
npm run dev &
FRONTEND_PID=$!
echo -e "  → Frontend PID: ${FRONTEND_PID}"
cd ../..

# Wait for frontend
sleep 5

# Start wind publisher
echo -e "${CYAN}[4/5] Starting wind publisher...${NC}"
bash scripts/run_wind.sh --x 4.0 --y 1.2 --z 0.0 --duration 60 >/dev/null 2>&1 &
WIND_PID=$!
echo -e "  → Wind PID: ${WIND_PID}"

# Monitor topics
echo -e "${CYAN}[5/5] Monitoring topics (10 seconds)...${NC}"
echo ""
echo -e "${GREEN}Wind topic data:${NC}"
timeout 2 ros2 topic echo /wind/force --once 2>/dev/null || echo "  (waiting for wind data...)"
echo ""

# Info
echo -e "${YELLOW}Dashboard is running:${NC}"
echo -e "  Backend:  http://localhost:8000"
echo -e "  Frontend: http://localhost:5173"
echo -e "  API Docs: http://localhost:8000/docs"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop${NC}"

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping all services...${NC}"

    if kill -0 "${WIND_PID}" 2>/dev/null; then
        kill "${WIND_PID}" 2>/dev/null || true
    fi

    if kill -0 "${FRONTEND_PID}" 2>/dev/null; then
        kill "${FRONTEND_PID}" 2>/dev/null || true
    fi

    if kill -0 "${BACKEND_PID}" 2>/dev/null; then
        kill "${BACKEND_PID}" 2>/dev/null || true
    fi

    echo -e "${GREEN}Done!${NC}"
}

trap cleanup EXIT INT TERM

# Wait
wait
