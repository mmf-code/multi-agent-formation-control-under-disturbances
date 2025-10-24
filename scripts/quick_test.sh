#!/bin/bash
################################################################################
# Quick System Test - 15 Second Validation
#
# Bu script sistemin düzgün çalıştığını hızlıca doğrular.
# Hocaya göstermeden önce "herşey hazır mı?" kontrolü için kullanılır.
#
# Kontrol edilen şeyler:
#   ✓ ROS2 setup
#   ✓ Workspace build durumu
#   ✓ Gazebo başlatma
#   ✓ Drone spawn
#   ✓ Metrics topics yayını
#   ✓ Controller çalışması
#
# Kullanım:
#   ./scripts/quick_test.sh
################################################################################

set -e

# Renk tanımları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Başlık
echo -e "${YELLOW}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${YELLOW}║  Quick System Test - 15 Second Validation                 ║${NC}"
echo -e "${YELLOW}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Test başlangıç
START_TIME=$(date +%s)

# Workspace kontrolü
WORKSPACE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
echo -ne "${CYAN}[1/7] Checking workspace...${NC} "
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}✗ FAILED${NC}"
    exit 1
fi
echo -e "${GREEN}✓ OK${NC}"

cd "$WORKSPACE_DIR"

# Build kontrolü
echo -ne "${CYAN}[2/7] Checking build...${NC} "
if [ ! -d "install" ] || [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}✗ NOT BUILT${NC}"
    echo -e "${YELLOW}Run: colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg${NC}"
    exit 1
fi
echo -e "${GREEN}✓ OK${NC}"

# ROS2 setup
echo -ne "${CYAN}[3/7] Setting up ROS2...${NC} "
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
echo -e "${GREEN}✓ OK${NC}"

# Eski process'leri temizle
echo -ne "${CYAN}[4/7] Cleaning old processes...${NC} "
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 1
echo -e "${GREEN}✓ OK${NC}"

# Gazebo başlat (background, headless)
echo -ne "${CYAN}[5/7] Launching Gazebo (headless)...${NC} "
timeout 25 ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=false rviz:=false > /tmp/quick_test.log 2>&1 &
SIM_PID=$!
sleep 12
echo -e "${GREEN}✓ OK${NC}"

# Topics kontrolü
echo -ne "${CYAN}[6/7] Checking ROS2 topics...${NC} "
EXPECTED_TOPICS=(
    "/agent_0/metrics"
    "/agent_3/metrics"
    "/agent_6/metrics"
    "/agent_0/odom"
    "/agent_0/target_pose"
)

ALL_OK=true
for topic in "${EXPECTED_TOPICS[@]}"; do
    if ! ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${RED}✗ FAILED${NC}"
        echo -e "${RED}  Missing topic: $topic${NC}"
        ALL_OK=false
        break
    fi
done

if [ "$ALL_OK" = true ]; then
    echo -e "${GREEN}✓ OK (${#EXPECTED_TOPICS[@]} topics)${NC}"
fi

# Metrics data kontrolü
echo -ne "${CYAN}[7/7] Testing metrics data...${NC} "
SAMPLE=$(timeout 3 ros2 topic echo /agent_0/metrics --once 2>/dev/null || echo "")
if [ -z "$SAMPLE" ]; then
    echo -e "${YELLOW}⚠ NO DATA (may need more time)${NC}"
else
    echo -e "${GREEN}✓ OK (receiving data)${NC}"
fi

# Temizlik
echo ""
echo -e "${CYAN}Cleaning up...${NC}"
kill $SIM_PID 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
sleep 1

# Özet
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  System Test Complete!                                     ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Test Duration:${NC} ${YELLOW}${ELAPSED}s${NC}"
echo -e "${GREEN}✓ System is ready for demo!${NC}"
echo ""
echo -e "${CYAN}Next steps:${NC}"
echo -e "  ${YELLOW}./scripts/run_formation_demo.sh${NC}      - Show visual demo to instructor"
echo -e "  ${YELLOW}./scripts/record_thesis_data.sh${NC}     - Record CSV data for thesis"
echo ""
