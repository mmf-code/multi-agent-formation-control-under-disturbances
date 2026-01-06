#!/bin/bash
# video_demo.sh - Tez video kaydı için interaktif simülasyon başlatıcı
#
# Kullanım: ./scripts/video_demo.sh [PHASE] [DURATION]
#   PHASE: 1-5 (varsayılan: 3 - rüzgarlı senaryo)
#   DURATION: saniye (varsayılan: 90)
#
# Özellikler:
#   - Figure-8 trajectory (3m radius loops)
#   - Synchronized formation movements (tüm gruplar aynı pattern)
#   - Shape transitions (triangle → line → v_shape → triangle)
#   - Height variations (0.5m → 0.9m → 0.5m)
#   - Wind disturbance (Phase 3+ için)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parametreler
PHASE=${1:-3}  # Default Phase 3 (rüzgarlı) - IT2/GT2 farkı görünür
DURATION=${2:-90}

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║     MULTI-AGENT FORMATION CONTROL - VIDEO DEMO               ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  Phase: $PHASE                                                 ║"
echo "║  Duration: ${DURATION}s                                            ║"
echo "║                                                              ║"
echo "║  Trajectory: Figure-8 pattern                                ║"
echo "║  Formations: Triangle → Line → V-Shape → Triangle            ║"
echo "║  Groups: PD (Y=-12) | PID (Y=-4) | IT2 (Y=4) | GT2 (Y=12)   ║"
echo "║                                                              ║"
echo "║  Timeline:                                                   ║"
echo "║    0-20s:  Assembly & stabilization                          ║"
echo "║    20-50s: Figure-8 with shape transitions                   ║"
echo "║    50-70s: Wind challenge (IT2/GT2 advantage visible)        ║"
echo "║    70-90s: Recovery & finale                                 ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# 1. Temizlik
echo "[1/5] Önceki simülasyon temizleniyor..."
"$SCRIPT_DIR/clean_sim.sh"
echo ""

# 2. ROS2 ortamını kaynak
echo "[2/5] ROS2 ortamı yükleniyor..."
source /opt/ros/humble/setup.bash
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
    echo "  ✓ Workspace yüklendi"
else
    echo "  ✗ HATA: install/setup.bash bulunamadı - colcon build çalıştırın"
    exit 1
fi

# Wayland uyumluluğu için X11 modu
export QT_QPA_PLATFORM=xcb

# 3. Çıkış klasörünü hazırla
OUTPUT_DIR="$PROJECT_ROOT/results/video_sessions/phase_${PHASE}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"
echo "  ✓ Çıktı: $OUTPUT_DIR"

# 4. Gazebo'yu duraklatılmış olarak başlat
echo ""
echo "[3/5] Gazebo başlatılıyor (duraklatılmış)..."

# Launch komutu - video_demo=true ile figure-8 trajectory kullanır
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=$PHASE \
    gazebo_gui:=true \
    output_dir:="$OUTPUT_DIR" \
    collision_mode:=disabled \
    paused:=true \
    video_demo:=true \
    duration:=$DURATION \
    2>&1 | tee "$OUTPUT_DIR/launch.log" &

LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# gzclient'in açılmasını bekle
echo ""
echo "[4/5] Gazebo yükleniyor..."
sleep 3

for i in {1..60}; do
    if pgrep -x gzclient > /dev/null 2>&1; then
        echo ""
        echo "  ✓ Gazebo GUI açıldı!"
        echo "  Modeller yükleniyor (5 saniye)..."
        sleep 5
        break
    fi
    printf "\r  Bekleniyor... (%d/60) " $i
    sleep 1
done

# Kullanıcının hazır olmasını bekle
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                    GAZEBO HAZIR                              ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  ✓ 12 drone yüklendi (4 grup × 3 drone)                      ║"
echo "║                                                              ║"
echo "║  Kamera Önerileri:                                           ║"
echo "║  • Top-down view: Tüm grupları görmek için                   ║"
echo "║  • Orbit view: 3D formation görünümü                         ║"
echo "║  • Follow mode: Tek grup takibi                              ║"
echo "║                                                              ║"
echo "║  Kayıt İpuçları:                                             ║"
echo "║  • OBS veya kazam ile ekran kaydı başlatın                   ║"
echo "║  • 1080p veya 4K önerilir                                    ║"
echo "║  • 30 fps yeterli                                            ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
read -p ">>> Simülasyonu başlatmak için ENTER'a basın... "

# 5. Simülasyonu başlat (unpause)
echo ""
echo "[5/5] Simülasyon başlatılıyor..."

# Gazebo'yu unpause yap (Gazebo Classic komutu)
gz world -p 0 2>/dev/null && echo "  ✓ Simülasyon başlatıldı" || echo "  ! gz komutu başarısız, Gazebo'da Play butonuna tıklayın"

# Progress bar göster
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                 SİMÜLASYON ÇALIŞIYOR                         ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Timeline gösterimi ile geri sayım
for ((i=0; i<=$DURATION; i++)); do
    # Progress bar
    progress=$((i * 50 / DURATION))
    bar=$(printf '%*s' "$progress" | tr ' ' '█')
    space=$(printf '%*s' "$((50 - progress))" | tr ' ' '░')

    # Faz gösterimi
    if [ $i -lt 20 ]; then
        phase_text="Assembly"
    elif [ $i -lt 50 ]; then
        phase_text="Figure-8 "
    elif [ $i -lt 70 ]; then
        phase_text="Wind Test"
    else
        phase_text="Finale   "
    fi

    printf "\r  [%s%s] %3ds/%ds | %s " "$bar" "$space" $i $DURATION "$phase_text"
    sleep 1
done
echo ""

# Simülasyonu durdur
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                     TAMAMLANDI                               ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  Çıktı klasörü: $OUTPUT_DIR"
echo "║  Log dosyası: $OUTPUT_DIR/launch.log"
echo "║                                                              ║"
echo "║  Sonraki adımlar:                                            ║"
echo "║  • python3 scripts/plot_phase_results.py ile grafik oluştur  ║"
echo "║  • Video düzenleme için kdenlive veya davinci önerilir       ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Temizlik
echo "Temizlik yapılıyor..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 1
"$SCRIPT_DIR/clean_sim.sh"

echo ""
echo "Video demo tamamlandı!"
