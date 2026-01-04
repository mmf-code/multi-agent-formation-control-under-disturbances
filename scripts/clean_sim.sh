#!/bin/bash
# clean_sim.sh - Simülasyon öncesi temizlik scripti (AGRESİF)
# Gazebo, ROS2 proseslerini, DDS kalıntılarını ve daemon'u temizler
#
# Kullanım: ./scripts/clean_sim.sh

echo "=== Simülasyon Temizliği (Agresif) ==="

# 1. TÜM ilgili prosesleri zorla kapat
echo ""
echo "[1/4] Prosesler sonlandırılıyor..."

# Gazebo
pkill -9 gzserver 2>/dev/null && echo "  - gzserver kapatıldı" || echo "  - gzserver: yok"
pkill -9 gzclient 2>/dev/null && echo "  - gzclient kapatıldı" || echo "  - gzclient: yok"
pkill -9 ruby 2>/dev/null || true  # Gazebo ruby scriptleri

# ROS2 prosesleri
pkill -9 -f "ros2 launch" 2>/dev/null && echo "  - ros2 launch kapatıldı" || echo "  - ros2 launch: yok"
pkill -9 -f "ros2 run" 2>/dev/null || true
pkill -9 -f "agent_controller_node" 2>/dev/null && echo "  - agent_controller kapatıldı" || true
pkill -9 -f "metrics_publisher_node" 2>/dev/null || true
pkill -9 -f "formation_coordinator" 2>/dev/null || true
pkill -9 -f "wind_publisher" 2>/dev/null || true
pkill -9 -f "phase_metrics_logger" 2>/dev/null || true

sleep 2

# 2. ROS2 Daemon restart (kritik!)
echo ""
echo "[2/4] ROS2 daemon yeniden başlatılıyor..."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true
echo "  - Daemon yeniden başlatıldı"

# 3. DDS/FastRTPS shared memory temizliği (zorla)
echo ""
echo "[3/4] DDS shared memory temizleniyor..."

# Tüm fastrtps dosyalarını sil (birden fazla deneme)
for i in 1 2 3; do
    rm -f /dev/shm/fastrtps_* 2>/dev/null
    rm -f /dev/shm/FastDDS* 2>/dev/null
done

FASTRTPS_COUNT=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l || echo "0")
echo "  - Kalan fastrtps dosyası: $FASTRTPS_COUNT"

# 4. Son kontrol
echo ""
echo "[4/4] Son durum kontrolü..."

PROCS_LEFT=0
pgrep -x gzserver > /dev/null 2>&1 && echo "  ! UYARI: gzserver hala çalışıyor" && PROCS_LEFT=1
pgrep -f "ros2 launch" > /dev/null 2>&1 && echo "  ! UYARI: ros2 launch hala çalışıyor" && PROCS_LEFT=1
pgrep -f "agent_controller" > /dev/null 2>&1 && echo "  ! UYARI: agent_controller hala çalışıyor" && PROCS_LEFT=1

SHM_LEFT=$(ls /dev/shm/fastrtps_* /dev/shm/FastDDS* 2>/dev/null | wc -l || echo "0")

echo ""
if [ "$PROCS_LEFT" -eq 0 ] && [ "$SHM_LEFT" -eq 0 ]; then
    echo "=== Temizlik tamamlandı - simülasyon başlatılabilir ==="
else
    echo "=== Temizlik tamamlandı (uyarılar var - tekrar çalıştır) ==="
fi
