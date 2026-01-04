# Simulation Guide: Multi-Agent Formation Control

Bu rehber, 12-drone simülasyon sisteminin nasıl çalıştırılacağını, phase testlerini ve sorun giderme adımlarını açıklar.

## İçindekiler

1. [Hızlı Başlangıç](#hızlı-başlangıç)
2. [Simülasyon Mimarisi](#simülasyon-mimarisi)
3. [Phase Test Sistemi](#phase-test-sistemi)
4. [Simülasyon Öncesi Kontroller](#simülasyon-öncesi-kontroller)
5. [Simülasyon Sonrası](#simülasyon-sonrası)
6. [Sorun Giderme](#sorun-giderme)
7. [Sonuçların Analizi](#sonuçların-analizi)

---

## Hızlı Başlangıç

```bash
# 1. Workspace'i source et
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. Temizlik yap (KRİTİK! Her simülasyon öncesi MUTLAKA çalıştır!)
./scripts/clean_sim.sh

# 3. Phase 1 (BASELINE) çalıştır - headless
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=false

# 4. Sonuçlar otomatik olarak results/phase_X/run_Y/ dizinine yazılır
```

**Tek satır kopyala-yapıştır komutları:**
```bash
# Phase 1 - BASELINE
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=false

# Phase 2 - STEADY_WIND
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=2 gazebo_gui:=false

# Phase 3 - TURBULENCE
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 gazebo_gui:=false

# Phase 4 - GUST
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=4 gazebo_gui:=false
```

---

## Simülasyon Mimarisi

### Drone Grupları (12 drone, 4 grup)

| Grup | Agents | Controller | Y-Lane | Açıklama |
|------|--------|------------|--------|----------|
| 0 | 0, 1, 2 | PD | -12m | Baseline (integral yok) |
| 1 | 3, 4, 5 | PID | -4m | Klasik PID |
| 2 | 6, 7, 8 | IT2-FLS | +4m | PID + Interval Type-2 Fuzzy |
| 3 | 9, 10, 11 | GT2-FLS | +12m | PID + General Type-2 Fuzzy |

### Kontrol Parametreleri

```yaml
PID Gains (tüm controller'lar için aynı):
  Kp: 3.501
  Ki: 1.946  (PD için 0)
  Kd: 3.608

Fuzzy Mix (Additive Mode):
  k_pid: 1.0      # %100 PID
  k_fuzzy: 0.5    # + %50 Fuzzy
  # Formula: u = 1.0*u_pid + 0.5*u_fuzzy

Wind Input:
  fuzzy.include_wind: true
  fuzzy.wind_scalar: 1.0
```

### Formasyon Yapısı

- **Shape**: Triangle (3 drone/grup)
- **Spacing**: 3.0 metre
- **Trajectory**: 4 waypoint, her biri 15 saniye
  - WP1: (-5, Y, 0.5) → t=0-15s
  - WP2: (0, Y, 0.8) → t=15-30s
  - WP3: (5, Y, 0.8) → t=30-45s
  - WP4: (5, Y, 0.5) → t=45-60s

---

## Phase Test Sistemi

### Phase Tanımları

| Phase | İsim | Süre | Rüzgar Profili | Amaç |
|-------|------|------|----------------|------|
| 1 | BASELINE | 60s | Yok (0 m/s) | Referans performans |
| 2 | STEADY_WIND | 60s | Sabit 3 m/s @ 45° | DC rejection |
| 3 | TURBULENCE | 60s | Von Karman TI=0.15 | Stokastik bozucu |
| 4 | GUST | 60s | 5 m/s ani rüzgar | Geçici yanıt |
| 5 | COMBINED | 60s | Türbülans + gust + yön | Kombine senaryo |
| 6 | ENDURANCE | 300s | Stokastik | Uzun süreli test |

### Çalıştırma Komutları

```bash
# Phase 1: BASELINE (referans)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=false

# Phase 2: STEADY_WIND (sabit rüzgar)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=2 gazebo_gui:=false

# Phase 3: TURBULENCE (türbülans - TI sweep)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 sweep_index:=0  # TI=0.15
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 sweep_index:=1  # TI=0.25
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 sweep_index:=2  # TI=0.35

# Phase 4: GUST (ani rüzgar)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=4 gazebo_gui:=false

# Phase 5: COMBINED (kombine)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=5 gazebo_gui:=false

# Phase 6: ENDURANCE (5 dakika)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=6 gazebo_gui:=false
```

### Ek Parametreler

```bash
# Tekrarlı deneyler için run_index
ros2 launch ... phase:=1 run_index:=2

# Reproducibility için seed
ros2 launch ... phase:=3 seed:=42

# GUI ile debug
ros2 launch ... phase:=1 gazebo_gui:=true

# Özel output dizini
ros2 launch ... phase:=1 output_dir:=my_results
```

---

## Simülasyon Öncesi Kontroller

### 1. Temizlik Script'i (KRİTİK - MUTLAKA ÇALIŞTIR!)

```bash
./scripts/clean_sim.sh
```

**ÖNEMLİ**: Bu script her simülasyon öncesi **MUTLAKA** çalıştırılmalı! Aksi takdirde:
- Drone'lar hareket etmeyebilir
- Sadece bazı gruplar veri üretebilir
- Simülasyon donabilir veya kasabilir

Bu script şunları yapar:
1. Tüm Gazebo ve ROS2 proseslerini zorla kapatır
2. **`ros2 daemon restart`** yapar (KRİTİK!)
3. `/dev/shm/fastrtps_*` DDS shared memory kalıntılarını temizler

**Neden kritik?** ROS2 daemon'u zamanla "kirlenebilir" ve node'lar birbirini göremez hale gelebilir. `ros2 daemon restart` bunu düzeltir.

### 2. Manuel Kontrol

```bash
# Çalışan Gazebo var mı?
pgrep -a gzserver

# Çalışan ROS2 prosesleri
pgrep -a ros2

# DDS shared memory durumu
ls /dev/shm/fastrtps_* 2>/dev/null | wc -l
```

### 3. Sistem Kaynakları

```bash
# RAM kullanımı
free -h

# CPU yükü
top -bn1 | head -5
```

### 4. Workspace Kontrolü

```bash
# Workspace source edilmiş mi?
echo $AMENT_PREFIX_PATH | grep -q "multi-agent" && echo "OK" || echo "Source install/setup.bash"

# Build güncel mi?
ls -la install/agent_control_pkg/lib/agent_control_pkg/agent_controller_node
```

---

## Simülasyon Sonrası

### Otomatik Çıktılar

Simülasyon bitince `results/phase_X/run_Y/` dizinine yazılır:

```
results/phase_1/run_1/
├── agent_0_pd.csv       # Agent 0 (PD) verileri
├── agent_1_pd.csv
├── agent_2_pd.csv
├── agent_3_pid.csv      # Agent 3 (PID) verileri
├── ...
├── agent_11_gt2.csv     # Agent 11 (GT2) verileri
├── group_summary.csv    # Grup bazlı özet metrikler
├── phase_metadata.json  # Phase bilgileri
└── wind_data.csv        # Rüzgar verileri
```

### Temel Metrikler

| Metrik | Açıklama | Formül |
|--------|----------|--------|
| RMSE | Root Mean Squared Error | sqrt(mean(e²)) |
| MAE | Mean Absolute Error | mean(\|e\|) |
| Peak Error | Maksimum hata | max(\|e\|) |
| Last Violation | Son 0.1m aşımı | max(t) where \|e\| > 0.1m |
| Control Effort | Kontrol harcaması | ∫\|u\| dt |

### Sonuçları Görüntüleme

```bash
# Grup özeti
cat results/phase_1/run_1/group_summary.csv | column -t -s,

# Belirli agent
head -20 results/phase_1/run_1/agent_0_pd.csv
```

---

## Sorun Giderme

### Problem: Simülasyon donuyor/kasıyor

**Sebep**: Arka planda kalmış prosesler veya DDS kalıntıları

**Çözüm**:
```bash
# 1. Temizlik script'i
./scripts/clean_sim.sh

# 2. Manuel temizlik (gerekirse)
pkill -9 gzserver gzclient
pkill -9 -f "ros2 launch"
rm -f /dev/shm/fastrtps_*

# 3. Tekrar dene
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=false
```

### Problem: "Unable to find model" hatası

**Sebep**: GAZEBO_MODEL_PATH ayarlanmamış

**Çözüm**:
```bash
source install/setup.bash
# veya
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/agent_control_pkg/share/agent_control_pkg/models
```

### Problem: Node'lar birbirini görmüyor

**Sebep**: DDS discovery problemi

**Çözüm**:
```bash
# DDS temizliği
rm -f /dev/shm/fastrtps_*
rm -f /dev/shm/FastDDS*

# ROS2 daemon restart
ros2 daemon stop
ros2 daemon start
```

### Problem: Controller parametreleri uygulanmıyor

**Kontrol**:
```bash
# Node başladığında logları izle
ros2 launch ... 2>&1 | grep -E "(Kp|Ki|Kd|mix)"
```

Beklenen çıktı:
```
PID[Kp=3.501, Ki=1.946, Kd=3.608], mix[k_pid=1.00, k_fuzzy=0.50]
```

### Problem: Metrikler kaydedilmiyor

**Kontrol**:
```bash
# phase_metrics_logger çalışıyor mu?
ros2 node list | grep phase_metrics

# Topic'ler aktif mi?
ros2 topic hz /agent_0/metrics
```

---

## Sonuçların Analizi

### Hızlı Karşılaştırma

```bash
# Tüm phase'lerin grup özeti
for p in 1 2 3 4 5; do
  echo "=== Phase $p ==="
  cat results/phase_$p/run_1/group_summary.csv | column -t -s, | head -5
  echo ""
done
```

### Python ile Analiz

```python
import pandas as pd

# Grup özeti yükle
df = pd.read_csv('results/phase_1/run_1/group_summary.csv')

# Controller'ları karşılaştır
print(df[['controller_type', 'mean_rmse', 'mean_mae', 'mean_peak_error']])

# En iyi controller
best = df.loc[df['mean_rmse'].idxmin()]
print(f"Winner: {best['controller_type']} with RMSE={best['mean_rmse']:.3f}m")
```

### Beklenen Sonuç Sıralaması

| Phase | Tipik Sıralama (En iyi → En kötü) |
|-------|-----------------------------------|
| BASELINE | GT2 < IT2 < PID < PD |
| STEADY_WIND | PID < IT2 ≈ GT2 < PD |
| TURBULENCE | IT2 < GT2 < PID < PD |
| GUST | IT2 < PID ≈ GT2 < PD |
| COMBINED | PID < GT2 < IT2 < PD |

**Not**: Fuzzy controller'lar (IT2, GT2) stokastik bozucularda daha iyi, sabit bozucularda PID'ye yakın veya biraz geride kalabilir.

---

## Yararlı Komutlar

```bash
# Canlı metrik izleme
ros2 topic echo /agent_0/metrics --once

# Rüzgar izleme
ros2 topic echo /wind/velocity --once

# Tüm topic'ler
ros2 topic list | grep -E "(metrics|target|odom)"

# Node bilgisi
ros2 node info /agent_0/agent_controller
```

---

*Son Güncelleme: 2026-01-04*
*Versiyon: 1.0*
