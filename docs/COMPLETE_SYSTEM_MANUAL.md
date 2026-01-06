# Multi-Agent Formation Control Under Disturbances
# Kapsamli Sistem Manueli ve Ogretici Rehber

**Versiyon:** 1.0
**Tarih:** 2026-01-05
**Yazar:** Claude Code - Tez Dokumantasyonu

---

## Icindekiler

### Ana Bolumler
1. [Sistem Genel Bakis](#1-sistem-genel-bakis)
2. [Mimari Yapi ve Katmanlar](#2-mimari-yapi-ve-katmanlar)
3. [ROS2 Node'lari ve Topic Yapisi](#3-ros2-nodelari-ve-topic-yapisi)
4. [Kontrolcu Algoritmalari](#4-kontrolcu-algoritmalari)
5. [Ruzgar Bozucu Modelleri](#5-ruzgar-bozucu-modelleri)
6. [5-Fazli Test Framework'u](#6-5-fazli-test-frameworku)
7. [Veri Akisi Diyagramlari](#7-veri-akisi-diyagramlari)
8. [Calistirma Adimlari](#8-calistirma-adimlari)
9. [Cikti ve Analiz](#9-cikti-ve-analiz)
10. [Dosya Referanslari](#10-dosya-referanslari)

### Ekler
- [Ek A: Hizli Referans Kartlari](#ek-a-hizli-referans-kartlari)
- [Ek A2: KOD OKUMA SIRASI VE OGRENME YOLU](#ek-a2-kod-okuma-sirasi-ve-ogrenme-yolu) **(ONEMLI!)**
  - [A2.1 Onerilen Okuma Sirasi](#a21-onerilen-okuma-sirasi-bottom-up-yaklasim)
  - [A2.2 ADIM 1: Mesaj Tipleri](#a22-adim-1-mesaj-tiplerini-anla)
  - [A2.3 ADIM 2: Gazebo Plugin](#a23-adim-2-gazebo-plugin-fizik-katmani)
  - [A2.4 ADIM 3: PID Controller](#a24-adim-3-pid-controller)
  - [A2.5 ADIM 4: IT2 Fuzzy](#a25-adim-4-it2-fuzzy-logic-system)
  - [A2.6 ADIM 5: GT2 Fuzzy](#a26-adim-5-gt2-fuzzy-logic-system)
  - [A2.7 ADIM 6: Agent Controller Node](#a27-adim-6-agent-controller-node-ros2-wrapper)
  - [A2.8 ADIM 7: Formation Coordinator](#a28-adim-7-formation-coordinator)
  - [A2.9 ADIM 8: Wind Publisher](#a29-adim-8-wind-publisher)
  - [A2.10 ADIM 9: Phase Metrics Logger](#a210-adim-9-phase-metrics-logger)
  - [A2.11 ADIM 10: Launch Dosyasi](#a211-adim-10-launch-dosyasi-hepsini-birlestiren)
  - [A2.12 Okuma Sirasi Tablosu](#a212-ozet-okuma-sirasi-tablosu)
  - [A2.13 Pratik Calisma Plani](#a213-pratik-calisma-plani)
- [Ek B: Waypoint Trajectory Sistemi](#ek-b-waypoint-trajectory-sistemi)
- [Ek C: Event-Triggered Communication (ETC)](#ek-c-event-triggered-communication-etc)
- [Ek D: Collision Avoidance](#ek-d-collision-avoidance-carpisma-onleme)
- [Ek E: Sorun Giderme (Troubleshooting)](#ek-e-sorun-giderme-troubleshooting)
- [Ek F: Performans Metrikleri Detayli Aciklama](#ek-f-performans-metrikleri-detayli-aciklama)
- [Ek G: Thesis Sonuclari Ozeti](#ek-g-thesis-sonuclari-ozeti)
- [Ek H: Referanslar ve Kaynaklar](#ek-h-referanslar-ve-kaynaklar)
- [Ek I: Hizli Baslatma Checklist](#ek-i-hizli-baslatma-checklist)

---

## 1. Sistem Genel Bakis

### 1.1 Proje Amaci

Bu proje, **12 adet Crazyflie 2.1 drone**'unun rugar bozuculari altinda formasyon kontrolunu test eder. 4 farkli kontrolcu tipi (PD, PID, IT2-FLS, GT2-FLS) karsilastirilarak, fuzzy mantik tabanli kontrolculerin klasik kontrolculere gore avantajlari analiz edilir.

### 1.2 Temel Ozellikler

```
+------------------------------------------------------------------+
|                    PROJE OZELLIKLERI                              |
+------------------------------------------------------------------+
| Drone Sayisi      | 12 (4 grup x 3 drone)                        |
| Kontrolcu Tipleri | PD, PID, IT2-FLS+PID, GT2-FLS+PID            |
| Ruzgar Modelleri  | von Karman, Dryden, Gust, Stochastic         |
| Test Fazlari      | 5 faz (BASELINE -> COMBINED)                 |
| Simulasyon        | Gazebo + ROS2 Humble                         |
| Kontrol Frekansi  | 200 Hz (dt=0.005s)                           |
| Metrik Toplama    | 10 Hz                                        |
| Toplam Sure       | 60s/faz (300s endurance)                     |
+------------------------------------------------------------------+
```

### 1.3 Drone Grup Yapisi

```
           Y-axis
             ^
             |
 +12m  [GT2-FLS Group 3: agent_9, 10, 11]  ----  Mor
             |
  +4m  [IT2-FLS Group 2: agent_6, 7, 8]    ----  Yesil
             |
  -4m  [PID Group 1: agent_3, 4, 5]        ----  Mavi
             |
 -12m  [PD Group 0: agent_0, 1, 2]         ----  Kirmizi
             |
 +-----------+-----------> X-axis
            Origin
```

---

## 2. Mimari Yapi ve Katmanlar

### 2.1 3-Katmanli Kontrol Hiyerarsisi

```
+=====================================================================+
|                    KATMAN 1: FORMATION COORDINATOR                   |
|  (formation_coordinator_node.cpp)                                    |
|                                                                      |
|  Gorev: Hedef pozisyon uretimi, waypoint yonetimi, PSO optimizasyon |
|  Girdi: Config YAML, waypoint listesi                               |
|  Cikti: /agent_X/target_pose (geometry_msgs/PoseStamped)            |
+=====================================================================+
                              |
                              | target_pose
                              v
+=====================================================================+
|                    KATMAN 2: AGENT CONTROLLER                        |
|  (agent_controller_node.cpp)                                         |
|                                                                      |
|  Gorev: Pozisyon hatasi hesaplama, PID/Fuzzy kontrol, cikti uretimi |
|  Girdi: target_pose, odom, wind/velocity                            |
|  Cikti: /agent_X/cmd_accel (geometry_msgs/Vector3)                  |
+=====================================================================+
                              |
                              | cmd_accel (m/s^2)
                              v
+=====================================================================+
|                    KATMAN 3: GAZEBO PLUGIN (SimpleDronePlugin)       |
|  (simple_drone_plugin.cpp)                                           |
|                                                                      |
|  Gorev: Fizik simulasyonu, kuvvet uygulama, odom yayinlama          |
|  Fizik: F = m * a_cmd, Z-axis altitude hold                         |
|  Cikti: /agent_X/odom (nav_msgs/Odometry)                           |
+=====================================================================+
```

### 2.2 Paket Yapisi

```
multi-agent-formation-control-under-disturbances/
|
+-- agent_control_pkg/                   # Ana kontrol paketi
|   +-- config/
|   |   +-- fuzzy_params_crazyflie.yaml  # IT2 fuzzy parametreleri
|   |   +-- gt2_fuzzy_params_crazyflie.yaml  # GT2 fuzzy parametreleri
|   |   +-- collision_avoidance_params.yaml
|   |   +-- ros2/
|   |       +-- agent_controller_default.yaml
|   |
|   +-- include/agent_control_pkg/
|   |   +-- pid_controller.hpp           # PID sinifi
|   |   +-- gt2_fuzzy_logic_system.hpp   # GT2 fuzzy sinifi
|   |   +-- it2_fuzzy_logic_system.hpp   # IT2 fuzzy sinifi
|   |
|   +-- src/
|   |   +-- pid_controller.cpp           # PID implementasyonu
|   |   +-- gt2_fuzzy_logic_system.cpp   # GT2 implementasyonu
|   |   +-- it2_fuzzy_logic_system.cpp   # IT2 implementasyonu
|   |   +-- ros/
|   |       +-- agent_controller_node.cpp  # Ana ROS2 node
|   |       +-- metrics_publisher_node.cpp # Metrik yayinlama
|   |
|   +-- scripts/
|   |   +-- wind_publisher.py            # Ruzgar yayinlama
|   |   +-- phase_metrics_logger.py      # Faz metrik kayit
|   |   +-- wind_turbulence_models.py    # von Karman/Dryden
|   |
|   +-- launch/
|   |   +-- phased_comparison.launch.py  # 5-fazli test launcher
|   |   +-- twelve_drone_comparison.launch.py
|   |
|   +-- plugins/
|   |   +-- simple_drone_plugin.cpp      # Gazebo plugin
|   |
|   +-- worlds/
|       +-- crazyflie_12drone_4group.world
|
+-- other_packages/
|   +-- formation_coordinator_pkg/       # Formasyon koordinasyon
|   |   +-- config/
|   |   |   +-- formation_12drone_pd.yaml
|   |   |   +-- formation_12drone_pid_group.yaml
|   |   |   +-- formation_12drone_it2_group.yaml
|   |   |   +-- formation_12drone_gt2_group.yaml
|   |   +-- src/
|   |       +-- formation_coordinator_node.cpp
|   |
|   +-- my_custom_interfaces_pkg/        # Ozel mesaj tipleri
|       +-- msg/
|           +-- MetricsData.msg          # Metrik verisi
|           +-- FormationState.msg       # Formasyon durumu
|
+-- scripts/
|   +-- generate_comprehensive_plots.py  # Grafik uretimi
|   +-- generate_thesis_figures.py
|
+-- docs/                                # Dokumantasyon
    +-- SIMULATION_GUIDE.md
    +-- SESSION_SUMMARY_2026-01-05.md
    +-- architecture/
    +-- dynamics/
```

---

## 3. ROS2 Node'lari ve Topic Yapisi

### 3.1 Node Listesi

| Node Ismi | Paket | Executable | Gorev |
|-----------|-------|------------|-------|
| `agent_controller` | agent_control_pkg | agent_controller_node | Pozisyon kontrolu |
| `metrics_publisher` | agent_control_pkg | metrics_publisher_node | Performans metrikleri |
| `formation_coordinator_X` | formation_coordinator_pkg | formation_coordinator_node | Hedef uretimi |
| `wind_publisher` | agent_control_pkg | wind_publisher.py | Ruzgar simulasyonu |
| `phase_metrics_logger` | agent_control_pkg | phase_metrics_logger.py | CSV kayit |

### 3.2 Topic Haritasi

```
+-------------------------------------------------------------------------+
|                         TOPIC YAPISI (12 Drone)                          |
+-------------------------------------------------------------------------+

FORMATION COORDINATOR --> AGENT CONTROLLER:
  /agent_X/target_pose  (geometry_msgs/PoseStamped)
    - Hedef pozisyon (x, y, z, orientation)
    - 10 Hz publish rate

GAZEBO PLUGIN --> AGENT CONTROLLER:
  /agent_X/odom  (nav_msgs/Odometry)
    - Gercek pozisyon ve hiz
    - ~1000 Hz (Gazebo physics rate)

AGENT CONTROLLER --> GAZEBO PLUGIN:
  /agent_X/cmd_accel  (geometry_msgs/Vector3)
    - Komut ivme (ax, ay, az)
    - 200 Hz (kontrol frekansi)

WIND PUBLISHER --> AGENT CONTROLLER & PLUGIN:
  /wind/velocity  (geometry_msgs/Vector3)
    - Ruzgar hizi (vx, vy, vz)
    - 20 Hz

METRICS PUBLISHER --> LOGGER:
  /agent_X/metrics  (my_custom_interfaces_pkg/MetricsData)
    - Hata, RMSE, ITAE, settling time
    - 10 Hz

FORMATION COORDINATOR --> MONITORING:
  /formation_X/state  (my_custom_interfaces_pkg/FormationState)
    - Shape, spacing, center
```

### 3.3 Detayli Topic Diyagrami

```
                    +-------------------+
                    |  wind_publisher   |
                    +--------+----------+
                             |
                             | /wind/velocity
                             v
+------------------+    +----+------------+    +------------------+
|   formation_     |    |                 |    |                  |
|   coordinator    +--->|  agent_         +--->|  Gazebo          |
|   (Group 0-3)    |    |  controller     |    |  SimpleDrone     |
|                  |    |  (per agent)    |    |  Plugin          |
+--------+---------+    +--------+--------+    +--------+---------+
         |                       |                      |
         | /agent_X/             | /agent_X/            | /agent_X/odom
         | target_pose           | cmd_accel            |
         v                       v                      v
+--------+---------+    +--------+--------+    +--------+---------+
|  10 Hz           |    |  200 Hz         |    |  ~1000 Hz        |
|  PoseStamped     |    |  Vector3        |    |  Odometry        |
+------------------+    +-----------------+    +------------------+
                                 |
                                 | Internal
                                 v
                        +--------+--------+
                        |  metrics_       |
                        |  publisher      |
                        +--------+--------+
                                 |
                                 | /agent_X/metrics
                                 v
                        +--------+--------+
                        |  phase_metrics_ |
                        |  logger         |
                        +--------+--------+
                                 |
                                 | CSV Export
                                 v
                        +------------------+
                        |  results/        |
                        |  phase_X/run_Y/  |
                        +------------------+
```

---

## 4. Kontrolcu Algoritmalari

### 4.1 Kontrol Sistemi Blok Diyagrami

```
                         KONTROL SISTEMI
+-------------------------------------------------------------------------+
|                                                                          |
|  target_pose     +-------+     error      +------------+                |
|  (r) -----+----->|  -    |--------------->|            |                |
|           |      +-------+                |  PID/Fuzzy |    u_total     |
|           |          ^                    |  Controller|--------------->|
|           |          | current_pos        |            |                |
|           |          |                    +-----+------+                |
|           |          |                          |                       |
|           |          |                          | (hybrid mode)         |
|           |          |                          v                       |
|           |          |                    +------------+                |
|           |          |                    |  Fuzzy     |                |
|           |          |                    |  Overlay   |                |
|           |          |                    +------------+                |
|           |          |                                                  |
|           |     +----+-----+                                            |
|           |     |  Gazebo  |<-------------------------------------------+
|           |     |  Plugin  |       cmd_accel                            |
|           |     +----------+                                            |
|           |          |                                                  |
|           |          | odom                                             |
|           +----------+                                                  |
|                                                                          |
+-------------------------------------------------------------------------+
```

### 4.2 PID Kontrolcu Formulasyonu

```
Continuous Time:
  u(t) = Kp * e(t) + Ki * integral(e(t)) + Kd * de/dt

Discrete Implementation (agent_control_pkg/src/pid_controller.cpp):
  P_term = Kp * error
  I_term = Ki * integral  (with anti-windup)
  D_term = -Kd * d(measurement)/dt  (derivative on measurement)

  u = P_term + I_term + D_term

Anti-Windup Modes:
  - CONDITIONAL: Stop integrating when saturated
  - BACK_CALCULATION: Reduce integral based on saturation
  - COMBINED: Both methods (default)
```

**Tuned PID Gains (Crazyflie 2.1):**
```yaml
pid.kp: 3.501   # Proportional gain
pid.ki: 1.946   # Integral gain (0 for PD)
pid.kd: 3.608   # Derivative gain
```

### 4.3 IT2-FLS (Interval Type-2 Fuzzy Logic System)

```
IT2-FLS PIPELINE:
+----------+     +------------+     +----------------+     +----------+
| Crisp    |---->| Fuzzify    |---->| Rule Inference |---->| Type     |
| Inputs   |     | (IT2 MFs)  |     | (21 Rules)     |     | Reduction|
| (e, de)  |     +------------+     +----------------+     | (K-M)    |
+----------+                                               +----+-----+
                                                                |
                                                                v
                                                          +----------+
                                                          | Defuzzify|
                                                          | (Average)|
                                                          +----+-----+
                                                                |
                                                                v
                                                          Crisp Output
                                                          (correction)

Input Variables:
  - error (e): [-5, 5] m
  - error_rate (de/dt): [-2, 2] m/s
  - wind (optional): [0, 5] m/s

Output:
  - correction: [-6, 6] m/s^2

Fuzzy Sets (Linguistic):
  NB (Negative Big), NS (Negative Small), ZO (Zero),
  PS (Positive Small), PB (Positive Big)

FOU (Footprint of Uncertainty):
  Upper MF: overline{mu}(x)
  Lower MF: underline{mu}(x)
```

**Rule Matrix (5x5 = 25 kuraldan 21'i aktif):**

```
        de
   e    | NB   NS   ZO   PS   PB
  ------+---------------------------
   NB   | NB   NB   NB   NS   ZO
   NS   | NB   NS   NS   ZO   PS
   ZO   | NS   NS   ZO   PS   PS
   PS   | NS   ZO   PS   PS   PB
   PB   | ZO   PS   PB   PB   PB
```

### 4.4 GT2-FLS (General Type-2 Fuzzy Logic System)

```
GT2-FLS vs IT2-FLS:
+------------------+-------------------+--------------------+
| Ozellik          | IT2-FLS           | GT2-FLS            |
+------------------+-------------------+--------------------+
| Secondary MF     | Uniform (interval)| Triangular/Gauss   |
| Alpha Levels     | 1 (implicit)      | 5 (configurable)   |
| Computation      | O(N log N)        | O(N x alpha_levels)|
| Uncertainty      | Medium            | High               |
| Noise Rejection  | Good              | Better             |
+------------------+-------------------+--------------------+

GT2 Parameters:
  gt2.num_alpha_levels: 5
  gt2.secondary_shape: triangular
  gt2.secondary_spread: 0.2
```

### 4.5 Hibrit Kontrol Modu (PID + Fuzzy)

```
ADDITIVE HYBRID MODE:
+-------------------------------------------------------------+
|                                                              |
|  u_total = k_pid * u_pid + k_fuzzy * u_fuzzy                |
|                                                              |
|  Where:                                                      |
|    k_pid   = 1.0  (full PID contribution)                   |
|    k_fuzzy = 0.8  (80% fuzzy contribution)                  |
|                                                              |
|  Philosophy:                                                 |
|    - PID handles DC rejection (steady-state error)          |
|    - Fuzzy handles transient disturbances (gusts, turbulence)|
|                                                              |
+-------------------------------------------------------------+
```

---

## 5. Ruzgar Bozucu Modelleri

### 5.1 Ruzgar Profil Turleri

```
+------------------------------------------------------------------+
|  PROFILE     | ACIKLAMA                    | KULLANIM ALANI       |
+------------------------------------------------------------------+
|  constant    | Sabit ruzgar (vx, vy)       | DC rejection testi   |
|  sinusoidal  | Periyodik degisim           | Frekans yaniti       |
|  step        | Ani basamak degisimi        | Step response        |
|  gust        | Periyodik ruzgar hamleleri  | Transient rejection  |
|  vonkarman   | MIL-F-8785C turbulans       | Atmosferik realizm   |
|  dryden      | NASA dryden modeli          | Alternatif turbulans |
|  stochastic  | Rastgele + gust + wandering | Belirsizlik testi    |
+------------------------------------------------------------------+
```

### 5.2 von Karman Turbulans Modeli

```
POWER SPECTRAL DENSITY (PSD):

Longitudinal (u-component):
                     2 * sigma_u^2 * L_u
  Phi_u(omega) = ---------------------------
                  pi * V * (1 + (L_u*omega/V)^2)^(5/6)

Lateral (v-component):
                sigma_v^2 * L_v   1 + (8/3)*(L_v*omega/V)^2
  Phi_v(omega) = --------------- * ---------------------------
                   pi * V        (1 + (L_v*omega/V)^2)^(11/6)

Parameters:
  sigma_u, sigma_v, sigma_w : Turbulence intensities [m/s]
  L_u, L_v, L_w             : Integral length scales [m]
  V                         : Mean wind speed [m/s]
  TI = sigma / V            : Turbulence Intensity (0.10-0.35)

Default Values:
  L_u = 30.0 m   (longitudinal)
  L_v = 15.0 m   (lateral)
  L_w = 5.0 m    (vertical)
  TI  = 0.15-0.35
```

### 5.3 Faz-Spesifik Ruzgar Konfigurasyonu

```
+------+---------------+------------------+---------------------------+
| Faz  | Isim          | Ruzgar Profili   | Parametreler              |
+------+---------------+------------------+---------------------------+
|  1   | BASELINE      | constant         | magnitude: 0.0 m/s        |
|  2   | STEADY_WIND   | constant         | magnitude: 3.0 m/s @ 45°  |
|  3   | TURBULENCE    | vonkarman        | TI: 0.15/0.25/0.35        |
|  4   | GUST          | gust             | 5.0 m/s, T=1.5s, I=10s    |
|  5   | COMBINED      | stochastic       | turbulence + gust + wander|
+------+---------------+------------------+---------------------------+
```

---

## 6. 5-Fazli Test Framework'u

### 6.1 Faz Akis Diyagrami

```
                    TEST EXECUTION FLOW
+------------------------------------------------------------------+

  Phase 1: BASELINE (60s)
  +------------------------+
  | No wind                |
  | Reference performance  |
  | Controller tuning      |
  +------------------------+
            |
            v
  Phase 2: STEADY_WIND (60s)
  +------------------------+
  | Constant 3 m/s @ 45°   |
  | DC rejection test      |
  | Integral action needed |
  +------------------------+
            |
            v
  Phase 3: TURBULENCE (60s)
  +------------------------+
  | von Karman TI sweep    |
  | Stochastic disturbance |
  | Fuzzy advantage test   |
  +------------------------+
            |
            v
  Phase 4: GUST (60s)
  +------------------------+
  | Periodic 5 m/s gusts   |
  | Transient response     |
  | dError sensitivity     |
  +------------------------+
            |
            v
  Phase 5: COMBINED (60s)
  +------------------------+
  | Stochastic everything  |
  | Real-world conditions  |
  | Final comparison       |
  +------------------------+
            |
            v
     CSV EXPORT + PLOT GENERATION
```

### 6.2 Metrik Toplama Zamanlama

```
TIMELINE (per phase):
+------------------------------------------------------------------+
|                                                                   |
|  0s          10s              15s                             60s |
|  |           |                |                               |   |
|  +-----------|----------------+-------------------------------+   |
|  ^           ^                ^                               ^   |
|  |           |                |                               |   |
|  Formation   Wind starts      Steady-state                    End |
|  stabilize   (delayed start)  analysis begins                     |
|                                                                   |
|  Metrics excluded:            Metrics included:                   |
|  t < 15s (transient)          t >= 15s (SS metrics)               |
|                                                                   |
+------------------------------------------------------------------+
```

---

## 7. Veri Akisi Diyagramlari

### 7.1 Tam Sistem Veri Akisi

```
+-------------------------------------------------------------------------+
|                        COMPLETE DATA FLOW                                |
+-------------------------------------------------------------------------+

[YAML Config Files]
      |
      v
+------------------+          +------------------+
| formation_       |          | wind_publisher   |
| coordinator      |          | (Python)         |
| (C++)            |          |                  |
+--------+---------+          +--------+---------+
         |                             |
         | /agent_X/target_pose        | /wind/velocity
         | (PoseStamped, 10Hz)         | (Vector3, 20Hz)
         |                             |
         v                             v
+--------+-----------------------------+--------+
|                                               |
|            agent_controller_node              |
|                  (C++)                        |
|                                               |
|  +----------+    +----------+    +----------+ |
|  | PID      |    | IT2-FLS  |    | GT2-FLS  | |
|  | Control  |    | Control  |    | Control  | |
|  +----+-----+    +----+-----+    +----+-----+ |
|       |              |              |         |
|       +------+-------+-------+------+         |
|              |                                |
|              v                                |
|       u_total = k_pid*u_pid + k_fuzzy*u_fuzzy|
|                                               |
+-------------------------+---------------------+
                          |
                          | /agent_X/cmd_accel
                          | (Vector3, 200Hz)
                          v
+-------------------------+---------------------+
|                                               |
|              Gazebo SimpleDronePlugin         |
|                                               |
|   F = m * a_cmd + F_drag + F_wind            |
|   Z-axis: Altitude hold (Kp=8, Kd=3)         |
|                                               |
+-------------------------+---------------------+
                          |
                          | /agent_X/odom
                          | (Odometry, ~1000Hz)
                          v
+-------------------------+---------------------+
|                                               |
|            metrics_publisher_node             |
|                                               |
|   - Error calculation                        |
|   - RMSE, IAE, ITAE computation              |
|   - Wind metrics                             |
|                                               |
+-------------------------+---------------------+
                          |
                          | /agent_X/metrics
                          | (MetricsData, 10Hz)
                          v
+-------------------------+---------------------+
|                                               |
|            phase_metrics_logger               |
|                (Python)                       |
|                                               |
|   - Time series storage                      |
|   - CSV export                               |
|   - Group summary                            |
|                                               |
+-------------------------+---------------------+
                          |
                          v
+-------------------------+---------------------+
|              results/phase_X/run_Y/           |
|                                               |
|   agent_0_pd.csv                             |
|   agent_3_pid.csv                            |
|   agent_6_it2.csv                            |
|   agent_9_gt2.csv                            |
|   group_summary.csv                          |
|   wind_data.csv                              |
|   phase_metadata.json                        |
|                                               |
+-----------------------------------------------+
                          |
                          v
+-------------------------+---------------------+
|       generate_comprehensive_plots.py         |
|                                               |
|   - RMSE Evolution                           |
|   - XYZ Error Decomposition                  |
|   - Trajectory 2D                            |
|   - Wind Profile                             |
|   - Cross-phase Comparison                   |
|                                               |
+-----------------------------------------------+
                          |
                          v
              PDF/PNG Thesis Figures
```

### 7.2 Mesaj Yapilari

```
MetricsData.msg (my_custom_interfaces_pkg):
+------------------------------------------------------------------+
| Field                      | Type    | Aciklama                  |
+------------------------------------------------------------------+
| current_x, y, z            | float64 | Gercek pozisyon [m]       |
| target_x, y, z             | float64 | Hedef pozisyon [m]        |
| error_x, y, z              | float64 | Pozisyon hatasi [m]       |
| error_magnitude            | float64 | |e| = sqrt(ex^2+ey^2+ez^2)|
| rmse_x, y, z, total        | float64 | Root Mean Square Error    |
| iae_x, iae_y               | float64 | Integral Absolute Error   |
| itae_x, itae_y             | float64 | Integral Time Abs Error   |
| settling_time              | float64 | Oturma zamani [s]         |
| is_settled                 | bool    | Oturdu mu?                |
| max_overshoot_x, y         | float64 | Maks asim [m]             |
| mission_rmse               | float64 | Toplam misyon RMSE        |
| wind_velocity_x, y, z      | float64 | Ruzgar hizi [m/s]         |
| wind_magnitude             | float64 | |wind|                    |
| disturbance_rejection_ratio| float64 | DRR = |error|/|wind|      |
+------------------------------------------------------------------+

FormationState.msg:
+------------------------------------------------------------------+
| Field                      | Type      | Aciklama                |
+------------------------------------------------------------------+
| shape                      | string    | triangle/line/square    |
| spacing                    | float64   | Ajan arasi mesafe [m]   |
| center_x, y, z             | float64   | Merkez pozisyon         |
| yaw_deg                    | float64   | Oryantasyon [derece]    |
| agent_ids                  | string[]  | Ajan listesi            |
+------------------------------------------------------------------+
```

---

## 8. Calistirma Adimlari

### 8.1 On Gereksinimler

```bash
# 1. ROS2 Humble kurulumu
sudo apt install ros-humble-desktop

# 2. Gazebo kurulumu
sudo apt install ros-humble-gazebo-ros-pkgs

# 3. Python bagimliliklari
pip3 install numpy pandas matplotlib

# 4. Workspace build
cd ~/multi-agent-formation-control-under-disturbances
colcon build --symlink-install
source install/setup.bash
```

### 8.2 Simulasyon Oncesi Temizlik (KRITIK!)

```bash
#!/bin/bash
# scripts/clean_sim.sh

# Calisir Gazebo ve ROS2 proseslerini kapat
pkill -9 gzserver gzclient
pkill -9 -f "ros2 launch"
pkill -9 -f "agent_controller"

# DDS shared memory temizligi
rm -f /dev/shm/fastrtps_*
rm -f /dev/shm/FastDDS*

# ROS2 daemon restart (KRITIK!)
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 1

echo "Simulation environment cleaned!"
```

### 8.3 Tek Faz Calistirma

```bash
# Source workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Temizlik (MUTLAKA!)
./scripts/clean_sim.sh

# Phase 1: BASELINE (ruzgarsiz referans)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=1 \
    gazebo_gui:=false \
    output_dir:=results

# Phase 2: STEADY_WIND (sabit ruzgar)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=2 \
    gazebo_gui:=false

# Phase 3: TURBULENCE (turbulans, TI=0.15)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=3 \
    sweep_index:=0

# Phase 3: TURBULENCE (turbulans, TI=0.25)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=3 \
    sweep_index:=1

# Phase 4: GUST (periyodik ruzgar hamlesi)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=4

# Phase 5: COMBINED (tum bozucular)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=5 \
    seed:=42
```

### 8.4 Tek Satirlik Kopyala-Yapistir Komutlari

```bash
# Phase 1
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=false

# Phase 2
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=2 gazebo_gui:=false

# Phase 3
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 gazebo_gui:=false

# Phase 4
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=4 gazebo_gui:=false

# Phase 5
source /opt/ros/humble/setup.bash && source install/setup.bash && ./scripts/clean_sim.sh && ros2 launch agent_control_pkg phased_comparison.launch.py phase:=5 gazebo_gui:=false
```

### 8.5 Canli Izleme Komutlari

```bash
# Aktif topic listesi
ros2 topic list | grep -E "(metrics|target|odom|wind)"

# Agent 0 metriklerini izle
ros2 topic echo /agent_0/metrics --once

# Ruzgar verisini izle
ros2 topic echo /wind/velocity --once

# Topic frekanslarini kontrol
ros2 topic hz /agent_0/odom          # ~1000 Hz
ros2 topic hz /agent_0/cmd_accel     # ~200 Hz
ros2 topic hz /agent_0/metrics       # ~10 Hz

# Node durumu
ros2 node list
ros2 node info /agent_0/agent_controller
```

---

## 9. Cikti ve Analiz

### 9.1 Cikti Dosya Yapisi

```
results/
+-- phase_1/
|   +-- run_1/
|       +-- agent_0_pd.csv      # Group 0 (PD)
|       +-- agent_1_pd.csv
|       +-- agent_2_pd.csv
|       +-- agent_3_pid.csv     # Group 1 (PID)
|       +-- agent_4_pid.csv
|       +-- agent_5_pid.csv
|       +-- agent_6_it2.csv     # Group 2 (IT2-FLS)
|       +-- agent_7_it2.csv
|       +-- agent_8_it2.csv
|       +-- agent_9_gt2.csv     # Group 3 (GT2-FLS)
|       +-- agent_10_gt2.csv
|       +-- agent_11_gt2.csv
|       +-- group_summary.csv   # Grup bazli ozet
|       +-- wind_data.csv       # Ruzgar zaman serisi
|       +-- phase_metadata.json # Faz meta bilgisi
|
+-- phase_2/
|   +-- run_1/
|       +-- ...
|
+-- thesis_final/
    +-- plots/
        +-- phase_1_comprehensive.pdf
        +-- phase_2_comprehensive.pdf
        +-- all_phases_comparison.pdf
```

### 9.2 CSV Sutun Aciklamalari

**Agent CSV (25 sutun):**
```
timestamp           : Simulasyon zamani [s]
current_x,y,z       : Gercek pozisyon [m]
target_x,y,z        : Hedef pozisyon [m]
error_x,y,z         : Pozisyon hatasi [m]
error_magnitude     : |error| [m]
rmse_x,y,z,total    : Root Mean Square Error [m]
iae_x,y             : Integral Absolute Error [m.s]
itae_x,y            : Integral Time Absolute Error [m.s^2]
settling_time       : Oturma zamani [s]
max_overshoot_x,y   : Maks asim [m]
velocity_x,y,z      : Hiz [m/s]
```

**Group Summary CSV:**
```
controller_type     : PD, PID, IT2, GT2
group_id            : 0, 1, 2, 3
agent_count         : 3
mean_rmse           : Grup ortalama RMSE [m]
std_rmse            : RMSE standart sapma
ss_rmse             : Steady-state RMSE (t>15s) [m]
control_effort_iae  : Kontrol eforu [m/s^2 * s]
```

### 9.3 Grafik Uretimi

```bash
# Kapsamli grafik uretimi
python3 scripts/generate_comprehensive_plots.py \
    --results-dir results/thesis_final \
    --output-dir results/thesis_final/plots \
    --format both

# Sadece belirli bir faz icin
python3 scripts/generate_comprehensive_plots.py \
    --results-dir results/thesis_final \
    --phase 3 \
    --format pdf
```

### 9.4 Uretilen Grafik Kategorileri

```
9 KATEGORI GRAFIK:
+------------------------------------------------------------------+
| Kategori    | Grafik                    | Aciklama               |
+------------------------------------------------------------------+
| PERFORMANS  | rmse_evolution            | RMSE zaman serisi      |
|             | xyz_error_decomposition   | X/Y/Z hata ayrimi      |
|             | ss_error_boxplot          | SS hata dagilimi       |
+------------------------------------------------------------------+
| GUVENLIK    | min_inter_agent_distance  | Min ajan arasi mesafe  |
|             | collision_risk_histogram  | Carpisma riski dagil.  |
+------------------------------------------------------------------+
| ENERJI      | control_effort            | Kontrol eforu (IAE)    |
|             | jerk_analysis             | Sarsinti analizi       |
+------------------------------------------------------------------+
| YORUNGE     | trajectory_2d             | Kus bakisi X-Y         |
|             | altitude_hold             | Z yukseklik koruma     |
+------------------------------------------------------------------+
| BOZUCU      | wind_profile              | Ruzgar zaman serisi    |
|             | wind_correlation          | Ruzgar-hata korel.     |
+------------------------------------------------------------------+
| KARSILASTIR | phase_comparison_heatmap  | Faz x Kontrolcu        |
|             | controller_ranking        | Kazanan sayisi         |
+------------------------------------------------------------------+
```

### 9.5 Beklenen Sonuc Siralamasi

```
+------+---------------+----------------------------------+
| Faz  | Isim          | Tipik Siralama (En iyi -> Kotu) |
+------+---------------+----------------------------------+
|  1   | BASELINE      | GT2 < IT2 < PID < PD             |
|  2   | STEADY_WIND   | PID < IT2 ~ GT2 < PD             |
|  3   | TURBULENCE    | IT2 < GT2 < PID < PD             |
|  4   | GUST          | IT2 < PID ~ GT2 < PD             |
|  5   | COMBINED      | GT2 < IT2 < PID < PD             |
+------+---------------+----------------------------------+

SONUC: Fuzzy kontrolculer (IT2 + GT2) 4/5 fazda PID'yi yendi!
```

---

## 10. Dosya Referanslari

### 10.1 Anahtar Kaynak Dosyalari

| Dosya | Yer | Gorev |
|-------|-----|-------|
| `pid_controller.cpp` | agent_control_pkg/src/ | PID implementasyonu |
| `it2_fuzzy_logic_system.cpp` | agent_control_pkg/src/ | IT2-FLS |
| `gt2_fuzzy_logic_system.cpp` | agent_control_pkg/src/ | GT2-FLS |
| `agent_controller_node.cpp` | agent_control_pkg/src/ros/ | ROS2 kontrolcu node |
| `formation_coordinator_node.cpp` | formation_coordinator_pkg/src/ | Hedef uretici |
| `simple_drone_plugin.cpp` | agent_control_pkg/plugins/ | Gazebo fizik |
| `wind_publisher.py` | agent_control_pkg/scripts/ | Ruzgar simulasyonu |
| `phase_metrics_logger.py` | agent_control_pkg/scripts/ | CSV kayit |
| `wind_turbulence_models.py` | agent_control_pkg/scripts/ | von Karman/Dryden |

### 10.2 Konfigürasyon Dosyalari

| Dosya | Yer | Icerik |
|-------|-----|--------|
| `fuzzy_params_crazyflie.yaml` | agent_control_pkg/config/ | IT2 MF parametreleri |
| `gt2_fuzzy_params_crazyflie.yaml` | agent_control_pkg/config/ | GT2 parametreleri |
| `formation_12drone_pd.yaml` | formation_coordinator_pkg/config/ | PD grup config |
| `formation_12drone_pid_group.yaml` | formation_coordinator_pkg/config/ | PID grup config |
| `formation_12drone_it2_group.yaml` | formation_coordinator_pkg/config/ | IT2 grup config |
| `formation_12drone_gt2_group.yaml` | formation_coordinator_pkg/config/ | GT2 grup config |

### 10.3 Launch Dosyalari

| Dosya | Yer | Kullanim |
|-------|-----|----------|
| `phased_comparison.launch.py` | agent_control_pkg/launch/ | 5-fazli test |
| `twelve_drone_comparison.launch.py` | agent_control_pkg/launch/ | 12 drone karsilastirma |

### 10.4 Dokumantasyon

| Dosya | Yer | Icerik |
|-------|-----|--------|
| `SIMULATION_GUIDE.md` | docs/ | Simulasyon kilavuzu |
| `ROS2_INTEGRATION.md` | docs/ros2/ | ROS2 entegrasyonu |
| `plant_model_thesis.md` | docs/dynamics/ | Tez plant modeli |
| `it2_fuzzy_system.md` | docs/dynamics/ | IT2-FLS formulasyon |
| `WIND_TEST_FRAMEWORK.md` | docs/ | Ruzgar test plani |
| `SESSION_SUMMARY_2026-01-05.md` | docs/ | Son oturum ozeti |

---

## Ek A: Hizli Referans Kartlari

### A.1 Komut Seti

```bash
# Build
colcon build --symlink-install

# Source
source /opt/ros/humble/setup.bash && source install/setup.bash

# Temizle
./scripts/clean_sim.sh

# Tek faz calistir
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=N

# Grafik uret
python3 scripts/generate_comprehensive_plots.py -r results -o plots -f both
```

### A.2 Kritik Parametreler

```yaml
# PID Tuning (Crazyflie)
pid.kp: 3.501
pid.ki: 1.946
pid.kd: 3.608

# Fuzzy Mix
mix.k_pid: 1.0
mix.k_fuzzy: 0.8

# Wind Scalar
fuzzy.wind_scalar: 1.0

# GT2 Alpha Levels
gt2.num_alpha_levels: 5
gt2.secondary_spread: 0.2
```

### A.3 Topic Referans

```
/agent_X/target_pose    - Hedef (10 Hz)
/agent_X/odom           - Gercek (1000 Hz)
/agent_X/cmd_accel      - Komut (200 Hz)
/agent_X/metrics        - Metrik (10 Hz)
/wind/velocity          - Ruzgar (20 Hz)
```

---

## Ek A2: KOD OKUMA SIRASI VE OGRENME YOLU

Bu bolum, projeyi sifirdan anlamak isteyen biri icin **hangi dosyalari hangi sirada okumasi** gerektigini aciklar.

### A2.1 Onerilen Okuma Sirasi (Bottom-Up Yaklasim)

```
KATMAN 0: MESAJ TIPLERI (Ne iletiliyor?)
         |
         v
KATMAN 1: GAZEBO PLUGIN (Fizik nasil calisiyor?)
         |
         v
KATMAN 2: KONTROLCULER (PID, Fuzzy nasil calisiyor?)
         |
         v
KATMAN 3: ROS2 NODE (Kontrolcu nasil ROS2'ye baglaniyor?)
         |
         v
KATMAN 4: FORMATION COORDINATOR (Hedefler nasil uretiliyor?)
         |
         v
KATMAN 5: WIND PUBLISHER (Bozucular nasil uretiliyor?)
         |
         v
KATMAN 6: METRICS LOGGER (Veriler nasil toplanıyor?)
         |
         v
KATMAN 7: LAUNCH DOSYASI (Hepsi nasil birlestirilyor?)
```

---

### A2.2 ADIM 1: Mesaj Tiplerini Anla

**Dosya:** `other_packages/my_custom_interfaces_pkg/msg/MetricsData.msg`

```
NEDEN ONCE BU?
- Sistemde ne tur veriler dolastigini anlamak icin
- Topic'lerde ne yayinlandigini bilmek icin
```

**Okunacak satirlar ve aciklamalar:**
```
Satir 4-9   : current_x/y/z, target_x/y/z  -> Pozisyon bilgisi
Satir 12-15 : error_x/y/z, error_magnitude -> Hata hesabi
Satir 18-21 : rmse_x/y/z, rmse_total       -> Performans metrigi
Satir 24-28 : iae, itae                    -> Integral metrikler
Satir 31-34 : settling_time, is_settled    -> Oturma analizi
Satir 43-50 : wind_* alanlari              -> Ruzgar metrikleri
```

**Anlasilmasi gereken konsept:**
- Bu mesaj, her drone'un performansini olcmek icin kullaniliyor
- 10 Hz frekansta `/agent_X/metrics` topic'ine yayinlaniyor

---

### A2.3 ADIM 2: Gazebo Plugin (Fizik Katmani)

**Dosya:** `agent_control_pkg/plugins/simple_drone_plugin.cpp`

```
NEDEN IKINCI?
- En alt katman - fizik simulasyonu
- cmd_accel -> Force donusumu burada
- Odom yayini burada
```

**Kritik satirlar:**
```cpp
// Satir 128: Wind topic subscription
// Subscribe to global /wind/velocity

// Satir 180-220: Odom publisher ve wind subscriber kurulumu
this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    "/" + this->namespace_ + "/odom", 10);

// Satir 250-280: Ana fizik dongusu (OnUpdate)
void SimpleDronePlugin::OnUpdate() {
  // 1. cmd_accel oku
  // 2. Kuvvet hesapla: F = m * a
  // 3. Wind kuvveti ekle
  // 4. Gazebo'ya uygula
  // 5. Odom yayinla
}

// Satir 350-380: Z-axis altitude hold (ayri PD kontrolcu)
// Sadece XY ekseni cmd_accel'den geliyor
// Z ekseni plugin icinde tutulyor (Kp=8, Kd=3)
```

**Anlasilmasi gereken konsept:**
```
cmd_accel (m/s^2)  -->  F = m * a  -->  Gazebo physics  -->  odom
     ^                                                         |
     |                                                         |
     +---------------------------------------------------------+
                        Feedback loop
```

---

### A2.4 ADIM 3: PID Controller

**Dosya:** `agent_control_pkg/src/pid_controller.cpp`

```
NEDEN UCUNCU?
- Temel kontrol algoritmasi
- Fuzzy sistemler bunu extend ediyor
```

**Kritik satirlar:**
```cpp
// Satir 10-43: Constructor - parametreler
PIDController::PIDController(double kp, double ki, double kd, ...)
  : kp_(kp), ki_(ki), kd_(kd), ...

// Satir 77-150: Ana hesaplama fonksiyonu
PIDTerms PIDController::calculate_with_terms(double current_value, double dt) {
  // Satir 84: Hata hesabi
  double error = setpoint_ - current_value;

  // Satir 89: P terimi
  double p_term = kp_ * error;

  // Satir 96-114: D terimi (derivative on measurement - kick onleme)
  double derivative_input_change = current_value - previous_measurement_;
  double d_term = -kd_ * (derivative_input_change / dt);

  // Satir 117-149: I terimi + Anti-windup
  // CONDITIONAL: Saturasyonda integrasyon durdur
  if ((preliminary_output >= output_max_ && error > 0.0) ||
      (preliminary_output <= output_min_ && error < 0.0)) {
    should_integrate = false;
  }
}
```

**Anlasilmasi gereken konsept:**
```
          error = target - current
                    |
     +--------------+--------------+
     |              |              |
     v              v              v
   P = Kp*e    I = Ki*∫e dt   D = -Kd*dy/dt
     |              |              |
     +--------------+--------------+
                    |
                    v
              u = P + I + D
                    |
                    v
               Anti-windup
               (saturasyon kontrolu)
```

---

### A2.5 ADIM 4: IT2 Fuzzy Logic System

**Dosya:** `agent_control_pkg/src/it2_fuzzy_logic_system.cpp`

```
NEDEN DORDUNCU?
- PID'yi extend eden ilk fuzzy sistem
- Type reduction (Karnik-Mendel) burada
```

**Kritik satirlar:**
```cpp
// Satir 50-100: Membership function tanimlari
// Upper MF ve Lower MF (FOU - Footprint of Uncertainty)
struct IT2MembershipFunction {
  double upper_left, upper_center, upper_right;  // Upper MF
  double lower_left, lower_center, lower_right;  // Lower MF
};

// Satir 150-200: Fuzzification
// error -> {NB, NS, ZO, PS, PB} uyelik dereceleri
double fuzzify(double value, const IT2MembershipFunction& mf) {
  double upper = triangular_mf(value, mf.upper_*);
  double lower = triangular_mf(value, mf.lower_*);
  return {upper, lower};
}

// Satir 250-350: Rule inference (21 kural)
// IF error IS NB AND derror IS NB THEN output IS NB
for (auto& rule : rules_) {
  double firing_upper = min(error_mf.upper, derror_mf.upper);
  double firing_lower = min(error_mf.lower, derror_mf.lower);
  // Consequent agregasyonu...
}

// Satir 400-500: Karnik-Mendel Type Reduction
// IT2 -> T1 donusumu (interval -> crisp)
double typeReduce(const FiredRules& rules) {
  // Iteratif algoritma: y_l ve y_r bul
  // Sonuc: [y_l, y_r] intervali
  return (y_l + y_r) / 2.0;  // Centroid
}
```

**Anlasilmasi gereken konsept:**
```
       error, derror
            |
            v
    +----------------+
    |  Fuzzification |  -> Upper/Lower MF degerleri
    +----------------+
            |
            v
    +----------------+
    | Rule Inference |  -> 21 kural, firing strength
    +----------------+
            |
            v
    +----------------+
    | Type Reduction |  -> Karnik-Mendel algoritma
    | (IT2 -> T1)    |     [y_l, y_r] -> y_crisp
    +----------------+
            |
            v
    +----------------+
    | Defuzzification|  -> (y_l + y_r) / 2
    +----------------+
            |
            v
      fuzzy_output (m/s^2)
```

---

### A2.6 ADIM 5: GT2 Fuzzy Logic System

**Dosya:** `agent_control_pkg/src/gt2_fuzzy_logic_system.cpp`

```
NEDEN BESINCI?
- IT2'yi extend ediyor
- Alpha-cut tabanli (5 seviye)
- Secondary MF (triangular/gaussian)
```

**Kritik fark (IT2 vs GT2):**
```
IT2: Secondary MF = uniform (interval)
     +-----+
     |     |  <- Tum noktalar esit agirlik
     +-----+
     y_l   y_r

GT2: Secondary MF = triangular/gaussian
       /\
      /  \    <- Merkeze yakin noktalar daha agir
     /    \
    +------+
    y_l  y_r
```

**Kritik satirlar:**
```cpp
// Satir 80-120: Alpha-cut seviyeleri
// 5 seviye: alpha = {0.0, 0.25, 0.5, 0.75, 1.0}
for (int i = 0; i < num_alpha_levels_; i++) {
  double alpha = i / (num_alpha_levels_ - 1.0);
  // Her alpha icin ayri IT2 type reduction
  auto [y_l, y_r] = typeReduceAtAlpha(alpha, rules);
  alpha_cuts.push_back({alpha, y_l, y_r});
}

// Satir 200-250: Wavy-slice type reduction
// Her alpha-cut'in agirlikli ortalamasi
double gt2_output = 0.0;
double weight_sum = 0.0;
for (auto& cut : alpha_cuts) {
  double weight = secondaryMF(cut.alpha);  // triangular weight
  gt2_output += weight * (cut.y_l + cut.y_r) / 2.0;
  weight_sum += weight;
}
return gt2_output / weight_sum;
```

---

### A2.7 ADIM 6: Agent Controller Node (ROS2 Wrapper)

**Dosya:** `agent_control_pkg/src/ros/agent_controller_node.cpp`

```
NEDEN ALTINCI?
- PID ve Fuzzy'yi ROS2'ye bagliyor
- Topic subscribe/publish burada
- Hibrit kontrol modu burada
```

**Kritik satirlar:**
```cpp
// Satir 50-100: Subscriber'lar
odom_sub_ = create_subscription<Odometry>(
    "odom", 10, std::bind(&AgentControllerNode::odomCallback, this, _1));

target_sub_ = create_subscription<PoseStamped>(
    "target_pose", 10, std::bind(&AgentControllerNode::targetCallback, this, _1));

wind_sub_ = create_subscription<Vector3>(
    "/wind/velocity", 10, ...);  // GLOBAL topic!

// Satir 150-200: Publisher
cmd_pub_ = create_publisher<Vector3>("cmd_accel", 10);

// Satir 250-350: Control loop (200 Hz timer)
void controlLoop() {
  // 1. Hata hesapla
  double error_x = target_x_ - current_x_;
  double error_y = target_y_ - current_y_;

  // 2. PID hesapla
  double u_pid_x = pid_x_.calculate(current_x_, dt);
  double u_pid_y = pid_y_.calculate(current_y_, dt);

  // 3. Fuzzy hesapla (eger aktifse)
  double u_fuzzy_x = 0.0, u_fuzzy_y = 0.0;
  if (fuzzy_enabled_) {
    u_fuzzy_x = fuzzy_->compute(error_x, derror_x, wind_mag);
    u_fuzzy_y = fuzzy_->compute(error_y, derror_y, wind_mag);
  }

  // 4. HIBRIT KONTROL - PID + Fuzzy
  double u_x = k_pid_ * u_pid_x + k_fuzzy_ * u_fuzzy_x;
  double u_y = k_pid_ * u_pid_y + k_fuzzy_ * u_fuzzy_y;

  // 5. Publish
  cmd_msg.x = u_x;
  cmd_msg.y = u_y;
  cmd_pub_->publish(cmd_msg);
}
```

**Anlasilmasi gereken konsept:**
```
/agent_X/target_pose     /agent_X/odom      /wind/velocity
        |                      |                  |
        v                      v                  v
+-------+----------------------+------------------+--------+
|                AGENT CONTROLLER NODE                      |
|                                                           |
|  target ----+                                             |
|             |     error                                   |
|  current ---+-----> [-] ----+-----> PID -----> u_pid     |
|                             |                    |        |
|                             +-----> Fuzzy -> u_fuzzy     |
|                                                  |        |
|                         u = k_pid*u_pid + k_fuzzy*u_fuzzy |
|                                        |                  |
+----------------------------------------+------------------+
                                         |
                                         v
                              /agent_X/cmd_accel
```

---

### A2.8 ADIM 7: Formation Coordinator

**Dosya:** `formation_coordinator_pkg/src/formation_coordinator_node.cpp`

```
NEDEN YEDINCI?
- Hedef pozisyonlari ureten ust katman
- Waypoint yonetimi burada
- Formasyon sekilleri burada
```

**Kritik satirlar:**
```cpp
// Satir 55-144: Formasyon pozisyon hesabi
vector<array<double,2>> generateFormationPositions(FormationShape shape, size_t count) {
  switch (shape) {
    case TRIANGLE:
      // Eskenar ucgen, merkez orijinde
      positions[0] = {0.0, (sqrt(3)/3) * spacing};      // Tepe
      positions[1] = {-spacing/2, -(sqrt(3)/6)*spacing}; // Sol alt
      positions[2] = {spacing/2, -(sqrt(3)/6)*spacing};  // Sag alt
      break;
    case LINE:
      // Yatay cizgi
      for (i = 0; i < count; i++)
        positions[i] = {(i - half_count) * spacing, 0.0};
      break;
    // ... SQUARE, V_SHAPE
  }
}

// Satir 300-400: Waypoint interpolasyonu
void timerCallback() {
  double t = getCurrentTime();

  // Hangi waypoint'ler arasindayiz?
  int wp_idx = findWaypointIndex(t);

  // Linear interpolasyon
  double alpha = (t - times[wp_idx]) / (times[wp_idx+1] - times[wp_idx]);
  double center_x = lerp(waypoints_x[wp_idx], waypoints_x[wp_idx+1], alpha);
  double center_y = lerp(waypoints_y[wp_idx], waypoints_y[wp_idx+1], alpha);

  // Her ajan icin hedef = center + formation_offset
  for (int i = 0; i < num_agents; i++) {
    auto offset = formation_positions[i];
    publishTarget(agent_ids[i], center_x + offset[0], center_y + offset[1]);
  }
}
```

**Anlasilmasi gereken konsept:**
```
Waypoints: [(-5,0.5), (0,0.8), (5,0.8), (5,0.5)]
Times:     [0s,       15s,     30s,     45s,     60s]

t=22.5s (15s ile 30s arasi, %50)
  |
  v
center = lerp(waypoint[1], waypoint[2], 0.5)
       = lerp((0,0.8), (5,0.8), 0.5)
       = (2.5, 0.8)
  |
  v
agent_0_target = center + triangle_offset[0]
               = (2.5, 0.8) + (0, 1.73)  # tepe noktasi
               = (2.5, 2.53)
```

---

### A2.9 ADIM 8: Wind Publisher

**Dosya:** `agent_control_pkg/scripts/wind_publisher.py`

```
NEDEN SEKIZINCI?
- Bozucu ureten bagimsiz node
- Farkli profiller (constant, gust, vonkarman, stochastic)
```

**Kritik satirlar:**
```python
# Satir 69-210: Sinif ve parametreler
class WindPublisher(Node):
    def __init__(self):
        self.profile = self.get_parameter('profile')  # constant/gust/vonkarman/...
        self.magnitude = self.get_parameter('magnitude')
        self.direction_deg = self.get_parameter('direction')

        # Publisher - GLOBAL TOPIC (tum drone'lar dinliyor)
        self.wind_pub = self.create_publisher(Vector3, '/wind/velocity', 10)

# Satir 240-340: Profil hesaplamalari
def publish_wind(self):
    if self.profile == 'constant':
        current_mag = self.magnitude

    elif self.profile == 'sinusoidal':
        current_mag = self.magnitude * (0.5 + 0.5 * sin(2*pi*f*t))

    elif self.profile == 'gust':
        # Periyodik gust'lar
        if time_since_last_gust < gust_duration:
            current_mag = self.magnitude * sin(pi * progress)
        else:
            current_mag = 0.0

    elif self.profile == 'vonkarman':
        # Turbulans modeli
        turb_sample = self.turbulence_generator.generate_sample()
        msg.x = mean_wind_x + turb_sample.u
        msg.y = mean_wind_y + turb_sample.v

    elif self.profile == 'stochastic':
        # Rastgele: base wind + gust + direction wander
        # ... (en karmasik profil)
```

---

### A2.10 ADIM 9: Phase Metrics Logger

**Dosya:** `agent_control_pkg/scripts/phase_metrics_logger.py`

```
NEDEN DOKUZUNCU?
- Veri toplama ve CSV export
- Offline analiz icin kritik
```

**Kritik satirlar:**
```python
# Satir 149-268: Node kurulumu
class PhaseMetricsLogger(Node):
    def __init__(self):
        # Her ajan icin metrics subscriber
        for i in range(num_agents):
            sub = create_subscription(
                MetricsData,
                f"/agent_{i}/metrics",
                lambda msg, aid=f"agent_{i}": self.metrics_callback(msg, aid),
                qos
            )

        # Wind data subscriber
        self.wind_sub = create_subscription(Vector3, "/wind/velocity", ...)

# Satir 277-325: Metrics callback (veri biriktirme)
def metrics_callback(self, msg, agent_id):
    am = self.agent_metrics[agent_id]
    am.timestamps.append(elapsed)
    am.errors_x.append(msg.error_x)
    am.errors_y.append(msg.error_y)
    am.current_x.append(msg.current_x)  # Trajectory icin
    am.velocity_x.append(msg.velocity_x)  # Jerk icin
    # ...

# Satir 558-625: CSV export
def export_agent_csv(self, am):
    with open(filename, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "current_x", "current_y", "current_z",
            "error_x", "error_y", "error_z", "error_magnitude",
            "rmse_x", "rmse_y", "rmse_total", ...
        ])
        for i in range(am.samples):
            writer.writerow([am.timestamps[i], am.current_x[i], ...])
```

---

### A2.11 ADIM 10: Launch Dosyasi (Hepsini Birlestiren)

**Dosya:** `agent_control_pkg/launch/phased_comparison.launch.py`

```
NEDEN SONUNCU?
- Tum node'lari baslatir
- Parametreleri dagitir
- Phase konfigurasyonunu belirler
```

**Kritik satirlar:**
```python
# Satir 62-98: Phase'e gore wind parametreleri
def get_phase_wind_params(phase_id, sweep_index, seed):
    phase_configs = {
        1: {"profile": "constant", "magnitude": 0.0},           # BASELINE
        2: {"profile": "constant", "magnitude": 3.0},           # STEADY
        3: {"profile": "vonkarman", "turbulence_intensity": TI}, # TURBULENCE
        4: {"profile": "gust", "magnitude": 5.0},               # GUST
        5: {"profile": "stochastic", ...},                       # COMBINED
    }

# Satir 150-250: Node baslat sirasi
def generate_launch_description():
    return LaunchDescription([
        # 1. Gazebo (fizik simulatoru)
        ExecuteProcess(cmd=['gazebo', world_file, ...]),

        # 2. Formation coordinators (4 grup)
        Node(package='formation_coordinator_pkg', ...),  # PD
        Node(package='formation_coordinator_pkg', ...),  # PID
        Node(package='formation_coordinator_pkg', ...),  # IT2
        Node(package='formation_coordinator_pkg', ...),  # GT2

        # 3. Agent controllers (12 drone)
        # (Gazebo plugin icinde, ayri node degil)

        # 4. Wind publisher (tek, global)
        Node(package='agent_control_pkg', executable='wind_publisher.py', ...),

        # 5. Phase metrics logger
        Node(package='agent_control_pkg', executable='phase_metrics_logger.py', ...),
    ])
```

---

### A2.12 OZET: OKUMA SIRASI TABLOSU

```
+----+------------------------+---------------------------+-------------------+
| #  | Dosya                  | Ne Ogreniyor?             | Bagimlilk        |
+----+------------------------+---------------------------+-------------------+
| 1  | MetricsData.msg        | Veri yapilari             | Yok               |
| 2  | simple_drone_plugin.cpp| Fizik, F=ma, odom         | Msg tipleri       |
| 3  | pid_controller.cpp     | PID algoritma             | Yok               |
| 4  | it2_fuzzy_*.cpp        | IT2 fuzzy, K-M reduction  | PID konsepti      |
| 5  | gt2_fuzzy_*.cpp        | GT2 fuzzy, alpha-cuts     | IT2 konsepti      |
| 6  | agent_controller_node  | ROS2 wrapper, hibrit mod  | PID + Fuzzy       |
| 7  | formation_coordinator  | Hedef uretimi, waypoints  | Yok               |
| 8  | wind_publisher.py      | Bozucu profilleri         | Yok               |
| 9  | phase_metrics_logger   | Veri toplama, CSV         | MetricsData       |
| 10 | phased_comparison.launch| Hepsini birlestirme       | Tum dosyalar     |
+----+------------------------+---------------------------+-------------------+
```

---

### A2.13 PRATIK CALISMA PLANI

```
GUN 1: Temel Yapilar
  [x] MetricsData.msg oku (15 dk)
  [x] simple_drone_plugin.cpp:180-320 oku (45 dk)
  [x] Topic'leri ros2 topic list ile incele

GUN 2: Kontrol Algoritmalari
  [x] pid_controller.cpp:77-150 oku (30 dk)
  [x] it2_fuzzy_logic_system.cpp:150-500 oku (1 saat)
  [x] gt2 ile IT2 farkini anla (30 dk)

GUN 3: ROS2 Entegrasyonu
  [x] agent_controller_node.cpp:150-350 oku (45 dk)
  [x] Hibrit kontrol formulunu anla: u = k_pid*u_pid + k_fuzzy*u_fuzzy
  [x] ros2 topic echo ile canli veri izle

GUN 4: Ust Katmanlar
  [x] formation_coordinator_node.cpp:55-400 oku (45 dk)
  [x] wind_publisher.py:240-450 oku (30 dk)
  [x] phase_metrics_logger.py:277-625 oku (30 dk)

GUN 5: Entegrasyon
  [x] phased_comparison.launch.py oku (30 dk)
  [x] Phase 1 calistir, CSV kontrol et
  [x] generate_comprehensive_plots.py ile grafik uret
```

---

## Ek B: Waypoint Trajectory Sistemi

### B.1 Waypoint Konfigurasyonu

Her grup ayni waypoint trajectory'sini takip eder (fair comparison):

```yaml
# formation_12drone_pid_group.yaml ornek
waypoints:
  enable: true
  times: [15.0, 30.0, 45.0, 60.0]   # Gecis zamanlari [s]
  x: [-5.0, 0.0, 5.0, 5.0]          # X pozisyonlari [m]
  y: [-4.0, -4.0, -4.0, -4.0]       # Y pozisyonlari (grup lane)
  z: [0.5, 0.8, 0.8, 0.5]           # Yukseklik [m]
```

### B.2 Trajectory Zaman Cizelgesi

```
      X-axis
        ^
        |
  5.0 --+---------------------------*---------*  WP3, WP4
        |                          /
  0.0 --+----------------*--------/
        |               /
 -5.0 --*--------------/                        WP1, WP2
        |
        +----+----+----+----+----+----+---> Time
             15s  30s  45s  60s

Waypoint  | t [s]  | X [m] | Y [m] | Z [m]
----------|--------|-------|-------|-------
WP1       | 0-15   | -5.0  | lane  | 0.5
WP2       | 15-30  | 0.0   | lane  | 0.8
WP3       | 30-45  | 5.0   | lane  | 0.8
WP4       | 45-60  | 5.0   | lane  | 0.5

Lane Y-pozisyonlari:
  Group 0 (PD):   Y = -12m
  Group 1 (PID):  Y = -4m
  Group 2 (IT2):  Y = +4m
  Group 3 (GT2):  Y = +12m
```

---

## Ek C: Event-Triggered Communication (ETC)

### C.1 ETC Konsepti

```
GELENEKSEL (Time-Triggered):
  - Sabit periyotta (10 Hz) target gonder
  - Gereksiz iletisim bandwidth kullanimi

EVENT-TRIGGERED:
  - Sadece "onemli" degisikliklerde gonder
  - Threshold tabanli tetikleme
  - Bandwidth tasarrufu (60-80% azalma)
```

### C.2 ETC Parametreleri

```yaml
etc:
  enable: false        # Simdilik devre disi (test icin)
  epsilon_pos: 0.05    # Pozisyon degisim esigi [m]
  min_period_sec: 0.02 # Min gonderim periyodu [s] (50 Hz max)
  max_period_sec: 0.5  # Max gonderim periyodu [s] (2 Hz min)
```

### C.3 ETC Tetikleme Kosulu

```
Tetikle IF:
  |current_target - last_sent_target| > epsilon_pos
  OR
  time_since_last_send > max_period_sec

Gonderme IF:
  time_since_last_send >= min_period_sec
```

### C.4 ETC Devre Disi Birakilma Nedeni

Session 2026-01-05'te ETC'nin beklenmeyen RMSE artisina yol actigi tespit edildi:
- **Problem:** Phase 2'de RMSE ~246m (beklenilen ~0.8m)
- **Kök Neden:** ETC gecikmesi + ruzgar bozucu = kumulatif hata
- **Cözüm:** Tum config dosyalarinda `etc.enable: false` yapildi

---

## Ek D: Collision Avoidance (Carpisma Onleme)

### D.1 APF (Artificial Potential Field) Yontemi

```
POTANSIYEL ALAN FORMULASYONU:

Attractive Potential (Hedefe cekim):
  U_att(q) = 0.5 * k_att * ||q - q_goal||^2

Repulsive Potential (Engellerden itme):
              | 0.5 * k_rep * (1/d - 1/d_0)^2    if d <= d_0
  U_rep(q) = |
              | 0                                 if d > d_0

Toplam Kuvvet:
  F = -grad(U_att) - sum(grad(U_rep))
```

### D.2 Collision Avoidance Modlari

```bash
# Devre disi (baseline karsilastirma icin)
ros2 launch ... collision_mode:=disabled

# Pass-through (sadece republish, avoidance yok)
ros2 launch ... collision_mode:=pass_through

# Aktif avoidance
ros2 launch ... collision_mode:=avoidance
```

### D.3 Guvenlik Parametreleri

```yaml
# collision_avoidance_params.yaml
collision:
  safety_distance: 0.8    # Guvenli mesafe [m]
  collision_distance: 0.5 # Carpisma esigi [m]
  repulsion_gain: 1.5     # Itme kazanci
  max_avoidance_force: 2.0 # Max itme kuvveti [m/s^2]
```

---

## Ek E: Sorun Giderme (Troubleshooting)

### E.1 Simulasyon Donuyor/Kasiyor

```bash
# Sebep: Arka planda kalmis prosesler

# Cozum:
./scripts/clean_sim.sh
# veya manuel:
pkill -9 gzserver gzclient
pkill -9 -f "ros2 launch"
rm -f /dev/shm/fastrtps_*
ros2 daemon restart
```

### E.2 Node'lar Birbirini Gormuyor

```bash
# Sebep: DDS discovery problemi

# Cozum:
rm -f /dev/shm/fastrtps_*
rm -f /dev/shm/FastDDS*
ros2 daemon stop
sleep 2
ros2 daemon start

# Kontrol:
ros2 topic list   # Topic'ler gorunuyor mu?
ros2 node list    # Node'lar gorunuyor mu?
```

### E.3 "Unable to find model" Hatasi

```bash
# Sebep: GAZEBO_MODEL_PATH ayarlanmamis

# Cozum:
source install/setup.bash
# veya manuel:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/agent_control_pkg/share/agent_control_pkg/models
```

### E.4 Metrikler Kaydedilmiyor

```bash
# Kontrol:
ros2 node list | grep phase_metrics    # Logger calisiyor mu?
ros2 topic hz /agent_0/metrics         # Topic aktif mi?

# Sebep: Logger baslamadan simulasyon bitti
# Cozum: Daha uzun duration veya manual kontrol
```

### E.5 Yuksek RMSE Degerleri (>1m)

```
Olasi Sebepler:
1. ETC aktif -> etc.enable: false yap
2. Wind too strong -> magnitude azalt
3. Fuzzy fighting PID -> k_fuzzy azalt
4. PID gains wrong -> Crazyflie defaults kullan

Kontrol:
ros2 topic echo /agent_0/metrics --once
# error_magnitude degerine bak
```

### E.6 Drone Hareket Etmiyor

```bash
# Kontrol siralama:
1. ros2 topic hz /agent_0/cmd_accel    # Komut geliyor mu?
2. ros2 topic hz /agent_0/odom         # Odom yayinlaniyor mu?
3. ros2 topic echo /agent_0/target_pose # Hedef dogru mu?

# Cozum:
# Genellikle clean_sim.sh + rebuild cozuyor
colcon build --packages-select agent_control_pkg
source install/setup.bash
```

---

## Ek F: Performans Metrikleri Detayli Aciklama

### F.1 RMSE (Root Mean Square Error)

```
RMSE = sqrt( (1/N) * sum(e_i^2) )

Burada:
  e_i = sqrt(ex_i^2 + ey_i^2 + ez_i^2)
  N = sample sayisi

Yorumlama:
  RMSE < 0.1m : Mukemmel
  RMSE < 0.3m : Iyi
  RMSE < 0.5m : Kabul edilebilir
  RMSE > 1.0m : Kotu
```

### F.2 ITAE (Integral Time Absolute Error)

```
ITAE = integral( t * |e(t)| dt )

Ozellik:
  - Gec hatalari daha agir cezalandirir
  - Settling time'a hassas
  - Controller tuning icin ideal
```

### F.3 SS-RMSE (Steady-State RMSE)

```
SS-RMSE = RMSE(t >= t_ss)

Burada:
  t_ss = 15s (steady-state baslangici)

Neden onemli:
  - Startup transient'i haric tutar
  - Fair controller karsilastirmasi
  - Gercek performansi yansitir
```

### F.4 DRR (Disturbance Rejection Ratio)

```
DRR = |error| / |wind|

Yorumlama:
  DRR < 0.3 : Cok iyi rejection
  DRR < 0.5 : Iyi rejection
  DRR > 1.0 : Kotu (hata ruzgardan buyuk)
```

### F.5 Control Effort (IAE of Control)

```
Control_Effort = integral( |u(t)| dt )

Burada:
  u(t) = sqrt(ax^2 + ay^2)  [m/s^2]

Ozellik:
  - Enerji tuketimini yansitir
  - Dusuk = daha verimli
  - Fuzzy genellikle benzer (extra maliyet yok)
```

---

## Ek G: Thesis Sonuclari Ozeti

### G.1 Kontrolcu Performans Karsilastirmasi

```
+------------------------------------------------------------------+
|             ORTALAMA SONUCLAR (5 Faz Uzerinden)                  |
+------------------------------------------------------------------+
| Controller | Avg SS-RMSE | Win Count | Best Phase               |
+------------------------------------------------------------------+
| PD         | 1.346 m     | 0/5       | -                        |
| PID        | 1.105 m     | 1/5       | Phase 4 (GUST)           |
| IT2-FLS    | 0.985 m     | 2/5       | Phase 2,3                |
| GT2-FLS    | 0.992 m     | 2/5       | Phase 1,5                |
+------------------------------------------------------------------+

SONUC: Fuzzy kontrolculer (IT2+GT2) toplam 4/5 fazda kazandi!
```

### G.2 Faz Bazli Detayli Sonuclar

```
Phase 1 (BASELINE):
  Winner: GT2 (0.390m)
  Yorum: Ruzgarsiz, fuzzy advantage minimal

Phase 2 (STEADY_WIND):
  Winner: IT2 (0.717m)
  Yorum: Sabit ruzgar, IT2 daha stabil

Phase 3 (TURBULENCE):
  Winner: IT2 (0.768m)
  Yorum: Stokastik, fuzzy transient rejection

Phase 4 (GUST):
  Winner: PID (0.653m)
  Yorum: Periyodik gust, PID integral action

Phase 5 (COMBINED):
  Winner: GT2 (0.685m)
  Yorum: Karmasik, GT2 uncertainty handling
```

### G.3 Kontrol Eforu Analizi

```
+------------------------------------------------------------------+
| Controller | Avg Control Effort (IAE) | Efficiency               |
+------------------------------------------------------------------+
| PD         | 520.3                    | Baseline                 |
| PID        | 545.7                    | +4.9% (integral action)  |
| IT2-FLS    | 558.2                    | +7.3% (fuzzy correction) |
| GT2-FLS    | 562.1                    | +8.0% (GT2 overhead)     |
+------------------------------------------------------------------+

SONUC: Fuzzy ~8% fazla enerji kullanarak ~27% daha iyi performans!
       (Enerji/performans trade-off pozitif)
```

---

## Ek H: Referanslar ve Kaynaklar

### H.1 Akademik Referanslar

1. Mendel, J.M. (2001). *Uncertain Rule-Based Fuzzy Logic Systems*. Prentice Hall.
2. Karnik, N.N., & Mendel, J.M. (1998). "Introduction to Type-2 Fuzzy Logic Systems". IEEE FUZZ.
3. Franklin, G.F., et al. (2015). *Feedback Control of Dynamic Systems* (7th ed.). Pearson.
4. MIL-F-8785C: Flying Qualities of Piloted Aircraft.
5. IEC 61400-1: Wind Turbines - Design Requirements.

### H.2 Kod Referanslari

- PID Controller: `agent_control_pkg/src/pid_controller.cpp:10-150`
- IT2-FLS: `agent_control_pkg/src/it2_fuzzy_logic_system.cpp`
- GT2-FLS: `agent_control_pkg/src/gt2_fuzzy_logic_system.cpp`
- Formation Coordinator: `formation_coordinator_pkg/src/formation_coordinator_node.cpp:146-183`
- Wind Publisher: `agent_control_pkg/scripts/wind_publisher.py:69-488`
- Phase Logger: `agent_control_pkg/scripts/phase_metrics_logger.py:149-800`

### H.3 Dokumantasyon Referanslari

- Session Summary: `docs/SESSION_SUMMARY_2026-01-05.md`
- Simulation Guide: `docs/SIMULATION_GUIDE.md`
- ROS2 Integration: `docs/ros2/ROS2_INTEGRATION.md`
- Plant Model: `docs/dynamics/plant_model_thesis.md`
- IT2 System: `docs/dynamics/it2_fuzzy_system.md`
- Wind Framework: `docs/WIND_TEST_FRAMEWORK.md`
- 12-Drone Results: `docs/12DRONE_4GROUP_COMPARISON_RESULTS.md`

---

## Ek I: Hizli Baslatma Checklist

```
[ ] 1. Workspace source edildi mi?
      source /opt/ros/humble/setup.bash
      source install/setup.bash

[ ] 2. clean_sim.sh calistirildi mi? (KRITIK!)
      ./scripts/clean_sim.sh

[ ] 3. Gazebo model path dogru mu?
      echo $GAZEBO_MODEL_PATH

[ ] 4. DDS clean mi?
      ls /dev/shm/fastrtps_* 2>/dev/null | wc -l  # 0 olmali

[ ] 5. ros2 daemon saglikli mi?
      ros2 topic list  # Hata yok

[ ] 6. Launch parametreleri dogru mu?
      phase:=1-5
      gazebo_gui:=false (headless icin)
      output_dir:=results

[ ] 7. Disk alani yeterli mi?
      df -h .  # En az 1GB bos olmali

[ ] 8. Simulasyon sonrasi:
      - results/phase_X/run_Y/ kontrol
      - group_summary.csv kontrol
      - generate_comprehensive_plots.py calistir
```

---

*Bu dokuman Claude Code tarafindan 2026-01-05 tarihinde olusturulmustur.*
*Guncelleme: 2. Tur eklemeler yapildi.*
*Versiyon: 1.1*
