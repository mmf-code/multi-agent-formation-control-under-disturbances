# Simülasyon Çalıştırma Kılavuzu

Multi-Agent Formation Control simülasyonu için tam entegre çalıştırma kılavuzu.

---

## 🚀 Hızlı Başlangıç (En Kolay Yöntem)

### 1. Workspace'i Build Et
```bash
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
colcon build --packages-select agent_control_pkg monitoring_dashboard
source install/setup.bash
```

### 2. Backend'i Başlat (Terminal 1)
```bash
cd monitoring_dashboard
./scripts/run_backend.sh
```
- Backend: http://localhost:8000
- WebSocket: ws://localhost:8000/ws
- Production UI: http://localhost:8000/ui

### 3. Frontend'i Başlat (Terminal 2 - Opsiyonel)
```bash
cd monitoring_dashboard/frontend
npm run dev
```
- Development UI: http://localhost:3000

### 4. Dashboard'dan Simülasyon Başlat
Tarayıcıda http://localhost:3000 veya http://localhost:8000/ui adresini aç ve:
- "Start Simulation" butonuna tıkla
- Mevcut scriptlerden birini seç
- Gerçek zamanlı verileri izle

---

## 📋 Manuel Simülasyon Başlatma

### Temel Demo (60 saniye)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_full_demo.sh
```

### Gelişmiş 5-Fazlı Demo (300 saniye + Checkpoint)
```bash
./scripts/run_long_thesis_demo.sh
```

**Opsiyonlar:**
- `--test-mode` : 1 dakika hızlı test
- `--headless` : GUI'siz, max hız
- `--no-rviz` : RViz olmadan
- `--duration 120` : Özel süre

**Örnekler:**
```bash
./scripts/run_long_thesis_demo.sh --test-mode
./scripts/run_long_thesis_demo.sh --headless --duration 600
```

### Formation Demo (Sadece simülasyon)
```bash
./scripts/run_formation_demo.sh
```

---

## 🎯 Simülasyon Tipleri

### 1. **run_full_demo.sh** - Basit Hızlı Demo
- ⏱️ 60 saniye
- 📊 3 grup × 3 drone = 9 drone
- 📁 CSV kayıt
- ✅ Gazebo + RViz (opsiyonel)

### 2. **run_long_thesis_demo.sh** - Thesis Demo (ÖNERİLEN)
- ⏱️ 300 saniye (5 dakika)
- 🎯 5 fazlı zigzag senaryo
- 💾 Her 60 saniyede checkpoint kaydı
- 📊 Gelişmiş metrics logging
- 🌪️ Aşamalı rüzgar profili
- 📁 Detaylı veri yapısı:
  ```
  thesis_data/YYYY-MM-DD/HH-MM-SS_long_scenario/
  ├── raw_data/          # Tam CSV dosyaları
  ├── checkpoints/       # Faz bazlı veriler
  │   ├── phase_1/
  │   ├── phase_2/
  │   ├── phase_3/
  │   ├── phase_4/
  │   └── phase_5/
  └── final_results/     # Özet ve analiz
  ```

### 3. **run_formation_demo.sh** - Sadece Simülasyon
- 🎮 Veri kaydı YOK
- 🚁 Gazebo görselleştirme
- ⚡ Hızlı test için

---

## 🛑 Simülasyonu Durdurma

### Dashboard'dan (Önerilen)
- "Stop Simulation" butonuna tıkla
- Tüm processler otomatik temizlenir

### Manuel
```bash
pkill -9 gzserver
pkill -9 gzclient
pkill -9 rviz2
```

---

## 🎛️ Kontrol Grupları

| Grup | Agent'lar | Kontrolcü | Renk | Özellik |
|------|-----------|-----------|------|---------|
| 0 | 0, 1, 2 | PID+Fuzzy | 🟣 Magenta | Rüzgar direnci |
| 1 | 3, 4, 5 | PD | 🔵 Cyan | Hızlı settling |
| 2 | 6, 7, 8 | PID | 🟡 Yellow | Dengeli |

---

## 📊 Veri Kayıtları

### Çıktılar
Tüm veriler `thesis_data/` klasöründe:
```
thesis_data/
└── YYYY-MM-DD/
    └── HH-MM-SS_demo_name/
        ├── agent_0_pidfuzzy.csv
        ├── agent_3_pd.csv
        ├── agent_6_pid.csv
        └── simulation.log
```

### CSV Formatı
```
timestamp,sim_time,position_error,velocity_error,control_effort_x,control_effort_y,control_effort_z,is_settled
```

---

## 🔧 Sorun Giderme

### Backend'e Veri Gelmiyor
1. ROS2 environment source edildi mi?
   ```bash
   echo $ROS_DISTRO  # humble olmalı
   ```

2. Backend ROS2 topic'lere subscribe oldu mu?
   Backend loglarını kontrol et:
   ```
   2025-11-22 14:40:20 - ros_bridge.subscriptions - INFO - Subscribed to /agent_0/metrics
   ```

3. Simülasyon çalışıyor mu?
   ```bash
   ros2 topic list | grep metrics
   ```

### Dashboard Simulation Stop Çalışmıyor
- Backend'i yeniden başlat
- Manuel kill komutlarını kullan
- Process ID'yi kontrol et: `ps aux | grep gzserver`

### Port Zaten Kullanımda
```bash
# Port 8000
pkill -9 -f "python3.*app.py"

# Port 3000
pkill -9 -f "vite"
```

---

## 📈 Performans Karşılaştırması

### Normal Şartlar (Rüzgar Yok/Az)
- **PD (Group 1)**: En hızlı settling time, düşük overshoot
- **PID+Fuzzy (Group 0)**: İyi genel performans
- **PID (Group 2)**: Dengeli, orta seviye

### Rüzgar Altında
- **PID+Fuzzy (Group 0)**: En iyi rüzgar rejeksiyonu
- **PID (Group 2)**: Orta seviye direnç
- **PD (Group 1)**: Steady-state error artabilir (Ki=0)

> **Not**: PD kontrolcüsü rüzgar olmayan/hafif senaryolarda daha iyi performans gösterebilir.

---

## 🌐 Dashboard Özellikleri

### Gerçek Zamanlı Görselleştirme
- ✅ 2D Formation Map (drone pozisyonları)
- ✅ Error Charts (position, velocity)
- ✅ RMSE Trend
- ✅ Wind Force/Velocity
- ✅ Controller Parameters
- ✅ ROS2 Topology Graph
- ✅ Event Log

### Simülasyon Kontrolü
- ✅ Start/Stop simulation
- ✅ Script seçimi
- ✅ Status monitoring
- ✅ WebSocket bağlantı durumu

---

## 🎓 Thesis Veri Toplama

### Önerilen İş Akışı
```bash
# 1. Backend başlat
cd monitoring_dashboard && ./scripts/run_backend.sh

# 2. Uzun senaryo çalıştır (5 dakika)
./scripts/run_long_thesis_demo.sh

# 3. Veriyi analiz et
ls -lh thesis_data/$(date +%Y-%m-%d)/
```

### Checkpoint Analizi
Her 60 saniyede kaydedilen veriler ile:
- Faz bazlı performans
- Zamana bağlı adaptasyon
- Rüzgar etkisi karşılaştırması

---

## ⚙️ Gelişmiş Ayarlar

### Özel Simülasyon Süresi
```bash
./scripts/run_full_demo.sh 120        # 2 dakika
./scripts/run_long_thesis_demo.sh --duration 600  # 10 dakika
```

### Launch Parametreleri
```bash
ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=true \
    rviz:=false \
    world_name:=formation_comparison
```

---

## 📝 Notlar

- **Backend** her zaman ilk başlatılmalı (ROS2 bridge için)
- **Frontend** opsiyonel (production build backend'de mevcut)
- **Veri kaydı** otomatik (run_full_demo.sh ve run_long_thesis_demo.sh ile)
- **Dashboard stop** fonksiyonu process group kill ile çalışır

---

**Son Güncelleme**: 2025-11-22
**ROS2 Version**: Humble
**Dashboard Backend**: FastAPI + WebSocket
**Frontend**: React + Vite
