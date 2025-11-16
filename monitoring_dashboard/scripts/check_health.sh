#!/bin/bash

# Health check script for monitoring dashboard
# Checks if backend is running and responsive

echo "================================================"
echo "  Monitoring Dashboard Health Check"
echo "================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if backend is running
echo "1. Checking backend server..."
BACKEND_URL="http://localhost:8000"

# Try to connect to root endpoint
if curl -s -f "${BACKEND_URL}/" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Backend server is running${NC}"

    # Get detailed health info
    echo ""
    echo "2. Checking ROS bridge status..."
    HEALTH=$(curl -s "${BACKEND_URL}/api/health")

    if [ $? -eq 0 ]; then
        echo "$HEALTH" | python3 -m json.tool

        # Check ROS initialization
        ROS_OK=$(echo "$HEALTH" | python3 -c "import sys, json; data=json.load(sys.stdin); print(data.get('ros_ok', False))")

        if [ "$ROS_OK" = "True" ]; then
            echo -e "\n${GREEN}✓ ROS2 is initialized and running${NC}"
        else
            echo -e "\n${YELLOW}⚠ ROS2 may not be fully initialized${NC}"
        fi
    fi

    # Check WebSocket endpoint
    echo ""
    echo "3. Checking WebSocket availability..."
    if curl -s -f "${BACKEND_URL}/ws" -H "Connection: Upgrade" -H "Upgrade: websocket" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ WebSocket endpoint is accessible${NC}"
    else
        echo -e "${YELLOW}⚠ WebSocket endpoint check inconclusive (normal for curl)${NC}"
    fi

    # Check frontend build
    echo ""
    echo "4. Checking frontend build..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    FRONTEND_DIST="$SCRIPT_DIR/../frontend/dist"

    if [ -d "$FRONTEND_DIST" ] && [ -f "$FRONTEND_DIST/index.html" ]; then
        echo -e "${GREEN}✓ Frontend is built and ready${NC}"
        echo "   Frontend accessible at: ${BACKEND_URL}/ui/"
    else
        echo -e "${YELLOW}⚠ Frontend not built. Run: cd frontend && npm run build${NC}"
    fi

else
    echo -e "${RED}✗ Backend server is not running${NC}"
    echo ""
    echo "To start the backend:"
    echo "  ./scripts/run_backend.sh"
    exit 1
fi

echo ""
echo "================================================"
echo -e "${GREEN}Health check complete!${NC}"
echo "================================================"
echo ""
echo "Dashboard URLs:"
echo "  API:        ${BACKEND_URL}/"
echo "  Status:     ${BACKEND_URL}/api/status"
echo "  Health:     ${BACKEND_URL}/api/health"
echo "  Frontend:   ${BACKEND_URL}/ui/"
echo ""
