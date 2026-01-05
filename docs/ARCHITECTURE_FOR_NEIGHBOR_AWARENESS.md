# Multi-Agent Formation Control - Architecture Documentation

Bu dokuman, sisteme "neighbor awareness" (komsu farkindanligi) ozelligi eklemek isteyen LLM/gelistiriciler icin hazirlanmistir.

---

## 1. MEVCUT SISTEM MIMARISI

### 1.1 Genel Yapi (3 Katmanli Kontrol)

```
                    CENTRALIZED
                        |
        +---------------+---------------+
        |                               |
        v                               v
+-------------------+         +-------------------+
| Formation         |         | Collision Safety  |
| Coordinator       |         | Layer (Opsiyonel) |
| (per group)       |         | (Global, merkezi) |
+-------------------+         +-------------------+
        |                               |
        | /agent_X/target_pose          | /agent_X/safe_target_pose
        |                               |
        +---------------+---------------+
                        |
                        v
              +-------------------+
              | Agent Controller  |  <-- HER AJAN ICIN AYRI NODE
              | (PD/PID/IT2/GT2)  |
              +-------------------+
                        |
                        | /agent_X/cmd_accel
                        v
              +-------------------+
              | Gazebo Plugin     |
              | (Fizik Simulasyon)|
              +-------------------+
```

### 1.2 Mevcut Node'lar ve Gorevleri

| Node | Paket | Gorevi |
|------|-------|--------|
| `formation_coordinator_node` | formation_coordinator_pkg | Her grup icin hedef pozisyon uretir |
| `agent_controller_node` | agent_control_pkg | Yerel kontrolcu (PID/Fuzzy) |
| `collision_safety_layer_node` | agent_control_pkg | APF tabanli carpizma onleme |
| `wind_publisher` | agent_control_pkg | Ruzgar bozucusu publish eder |
| `metrics_publisher_node` | agent_control_pkg | Performans metriklerini toplar |

---

## 2. FORMATION COORDINATOR DETAYI

### 2.1 Dosya Konumlari

```
other_packages/formation_coordinator_pkg/
├── include/formation_coordinator_pkg/
│   └── formation_coordinator_node.hpp    # Header
├── src/
│   └── formation_coordinator_node.cpp    # Implementation
└── config/
    ├── formation_12drone_pd.yaml         # PD grubu (agents 0-2)
    ├── formation_12drone_pid_group.yaml  # PID grubu (agents 3-5)
    ├── formation_12drone_it2_group.yaml  # IT2 grubu (agents 6-8)
    └── formation_12drone_gt2_group.yaml  # GT2 grubu (agents 9-11)
```

### 2.2 Mevcut Yayinlanan Topic'ler (Her Coordinator Icin)

```cpp
// Formation Coordinator her ajan icin sadece target_pose yayinlar:
Publisher: /{agent_id}/target_pose  (geometry_msgs/PoseStamped)

// Ornek: formation_coordinator_it2 su topic'leri yayinlar:
/agent_6/target_pose
/agent_7/target_pose
/agent_8/target_pose
```

### 2.3 Coordinator YAML Config Ornegi

```yaml
# formation_12drone_it2_group.yaml
formation_coordinator_node:
  ros__parameters:
    agent_ids: ["agent_6", "agent_7", "agent_8"]
    formation:
      shape: "triangle"
      spacing: 2.0
      center:
        x: 5.0
        y: 0.0
        z: 1.0

    # ETC (Event-Triggered Communication)
    etc:
      enable: false
      epsilon_pos: 0.05
      min_period_sec: 0.02
      max_period_sec: 0.5

    # Waypoints (trajectory)
    waypoints:
      enable: true
      times: [15.0, 30.0, 45.0, 60.0]
      x: [5.0, 15.0, 25.0, 35.0]
      y: [0.0, 5.0, 0.0, -5.0]
      z: [1.0, 1.0, 1.0, 1.0]
      shapes: ["triangle", "line", "v_shape", "triangle"]
```

### 2.4 Formation Coordinator'in BILMEDIGI Seyler

**KRITIK:** Mevcut coordinator asagidaki bilgilere SAHIP DEGIL:
- Diger grup ajanlarin konumlari
- Kendi grubundaki ajanlarin GERCEK (odom) konumlari
- Komsu ajan bilgisi

Coordinator sadece:
- Kendi grubundaki ajan ID'lerini bilir
- Formasyon seklini ve hedef merkezi bilir
- Her ajana offset'li hedef pozisyon hesaplar

---

## 3. AGENT CONTROLLER DETAYI

### 3.1 Dosya Konumlari

```
agent_control_pkg/
├── include/agent_control_pkg/
│   └── agent_controller_node.hpp
├── src/ros/
│   └── agent_controller_node.cpp
└── config/ros2/
    └── agent_controller_default.yaml
```

### 3.2 Agent Controller'in Subscribe Ettigi Topic'ler

```cpp
// agent_controller_node.cpp (lines 46-56)

// Hedef pozisyon (formation coordinator'dan)
target_sub_ = create_subscription<PoseStamped>("target_pose", 10, ...);

// Kendi pozisyonu (Gazebo plugin'den)
odom_sub_ = create_subscription<Odometry>("odom", SensorDataQoS, ...);

// Ruzgar bilgisi (global)
wind_sub_ = create_subscription<Vector3>("/wind/velocity", 10, ...);
```

### 3.3 Agent Controller'in BILMEDIGI Seyler

**KRITIK:** Mevcut agent controller asagidaki bilgilere SAHIP DEGIL:
- Diger ajanlarin konumlari
- Komsu ajan bilgisi
- Formasyon bilgisi (sadece kendi hedefini bilir)

Agent controller tamamen IZOLE calisir - sadece:
- Kendi hedefini (target_pose)
- Kendi konumunu (odom)
- Ruzgar bilgisini (wind/velocity)

---

## 4. COLLISION SAFETY LAYER DETAYI

### 4.1 Dosya Konumlari

```
agent_control_pkg/
├── include/agent_control_pkg/
│   └── collision_safety_layer.hpp
├── src/ros/
│   └── collision_safety_layer_node.cpp
└── config/
    └── collision_avoidance_params.yaml
```

### 4.2 Bu Layer ZATEN "Neighbor Awareness" Yapiyor

```cpp
// collision_safety_layer_node.cpp (lines 129-158)

void CollisionSafetyLayer::setupSubscribersAndPublishers()
{
  for (size_t i = 0; i < num_agents_; ++i) {
    const std::string agent_ns = "/agent_" + std::to_string(i);

    // TUM AJANLARIN ODOM'UNU DINLIYOR (neighbor awareness!)
    odom_subs_.push_back(
      create_subscription<Odometry>(agent_ns + "/odom", ...));

    // TUM AJANLARIN TARGET'INI DINLIYOR
    target_subs_.push_back(
      create_subscription<PoseStamped>(agent_ns + "/target_pose", ...));

    // HER AJAN ICIN SAFE_TARGET YAYINLIYOR
    safe_target_pubs_.push_back(
      create_publisher<PoseStamped>(agent_ns + "/safe_target_pose", ...));
  }
}
```

### 4.3 APF (Artificial Potential Field) Hesaplama

```cpp
// collision_safety_layer_node.cpp (lines 211-253)

Position2D CollisionSafetyLayer::computeRepulsion(size_t agent_idx) const
{
  Position2D total_repulsion(0.0, 0.0);
  const auto& my_state = agent_states_[agent_idx];

  for (size_t j = 0; j < num_agents_; ++j) {
    if (j == agent_idx) continue;  // Kendini atla

    const auto& other_state = agent_states_[j];

    // Diger ajana uzaklik
    Position2D diff = my_state.position - other_state.position;
    double dist = diff.norm();

    if (dist < influence_distance_) {
      // APF formulu: F_rep = k * (1/d - 1/d_inf)^2 * direction
      double magnitude = k_repulsion_ * pow(1/dist - 1/influence_distance_, 2);
      Position2D direction = diff.normalized();
      total_repulsion = total_repulsion + direction * magnitude;
    }
  }
  return total_repulsion;
}
```

### 4.4 Collision Layer Modlari

| Mod | Davranis |
|-----|----------|
| `disabled` | Hicbir sey yapmaz, safe_target publish etmez |
| `pass_through` | target_pose'u degistirmeden safe_target olarak publish eder |
| `avoidance` | APF uygular, safe_target = target + repulsion |

---

## 5. TOPIC YAPISI (12 Drone Senaryosu)

### 5.1 Mevcut Topic'ler

```
# Her ajan icin (X = 0-11):
/agent_X/odom                 # Gazebo -> Controller (pozisyon/hiz)
/agent_X/target_pose          # Coordinator -> Controller (hedef)
/agent_X/safe_target_pose     # Safety Layer -> Controller (guvenli hedef)
/agent_X/cmd_accel            # Controller -> Gazebo (kontrol cikisi)
/agent_X/metrics              # Metrics Publisher (performans)
/agent_X/diagnostics          # Controller (debug)
/agent_X/controller_params    # Controller (parametre bilgisi)

# Global:
/wind/velocity                # Wind Publisher (ruzgar hizi)
/collision_safety/metrics     # Safety Layer (carpizma metrikleri)
/formation_X/state            # Coordinator (formasyon durumu)
/formation_X/etc_metrics      # Coordinator (ETC metrikleri)
```

### 5.2 Grup Yapisi (Phased Comparison)

```
Group 0 (PD):   agents 0, 1, 2   -> formation_coordinator_pd
Group 1 (PID):  agents 3, 4, 5   -> formation_coordinator_pid
Group 2 (IT2):  agents 6, 7, 8   -> formation_coordinator_it2
Group 3 (GT2):  agents 9, 10, 11 -> formation_coordinator_gt2
```

---

## 6. NEIGHBOR AWARENESS EKLEME SECENEKLERI

### 6.1 Secenek A: Formation Coordinator'a Neighbor Info Ekleme (ONERILEN)

**Fikir:** Coordinator her ajana, komsu ajanlarin konumlarini da gondersin.

**Yeni Topic:**
```
/agent_X/neighbor_info  (my_custom_interfaces/NeighborInfo)
```

**Yeni Mesaj Tipi:**
```
# my_custom_interfaces_pkg/msg/NeighborInfo.msg
std_msgs/Header header
string agent_id
NeighborPosition[] neighbors

# NeighborPosition.msg
string neighbor_id
float64 x
float64 y
float64 z
float64 distance
```

**Degisiklik Gereken Dosyalar:**
1. `my_custom_interfaces_pkg/msg/NeighborInfo.msg` (YENI)
2. `formation_coordinator_node.hpp` - subscriber/publisher ekle
3. `formation_coordinator_node.cpp` - neighbor info hesapla ve publish et
4. `agent_controller_node.hpp` - subscriber ekle (opsiyonel)
5. `agent_controller_node.cpp` - neighbor info al ve logla (opsiyonel)

**Avantajlar:**
- Minimum degisiklik
- Mevcut yapiyi bozmaz
- Controller'a neighbor bilgisi ulasir
- Collision layer hala bagimsiz calisabilir

**Dezavantajlar:**
- Hala merkezi bilgi dagitimi (decentralized degil)

### 6.2 Secenek B: Decentralized Subscribe (Her Ajan Diger Ajanlari Dinler)

**Fikir:** Her agent_controller, diger N-1 ajanin odom'unu subscribe etsin.

**Degisiklik Gereken Dosyalar:**
1. `agent_controller_node.hpp` - neighbor_odom_subs_ vector ekle
2. `agent_controller_node.cpp` - N-1 subscription kur, neighbor konumlarini takip et

**Avantajlar:**
- Gercek decentralized awareness
- Her ajan kendi komsu bilgisini toplar

**Dezavantajlar:**
- N*(N-1) = 12*11 = 132 subscription (12 drone icin)
- Karmasiklik artar
- Olceklenebilirlik sorunu

### 6.3 Secenek C: Coordinator Odom Dinleyerek Neighbor Info Uretsin

**Fikir:** Formation coordinator, kendi grubundaki ajanlarin odom'unu dinlesin ve neighbor info uretsin.

**Yeni Subscribe:**
```cpp
// Formation coordinator her ajan icin odom dinler:
/agent_X/odom -> formation_coordinator
```

**Avantajlar:**
- Sadece grup icindeki ajanlar icin neighbor info
- Daha az subscription
- Gercek pozisyon bazli

**Dezavantajlar:**
- Coordinator daha karmasik hale gelir
- Cross-group neighbor info yok

---

## 7. MEVCUT ETC (Event-Triggered Communication) SISTEMI

### 7.1 ETC Nedir?

Formation coordinator'da zaten bir ETC sistemi var. Bu, her cycle'da degil, sadece gerektiginde target_pose yayinlar.

```cpp
// formation_coordinator_node.cpp (lines 677-766)

bool FormationCoordinatorNode::shouldTriggerEvent(
  const std::string& agent_id,
  const geometry_msgs::msg::PoseStamped& current_pose)
{
  // Event kosullari:
  // 1. Anti-chattering: min_period gecmediyse PUBLISH ETME
  // 2. Force: waypoint/shape degisiminde KESINLIKLE PUBLISH
  // 3. Position: hedef epsilon'dan fazla degistiyse PUBLISH
  // 4. Heartbeat: max_period gectiyse PUBLISH (stale onleme)
}
```

### 7.2 ETC Parametreleri

```yaml
etc:
  enable: true/false
  epsilon_pos: 0.05      # [m] pozisyon degisim esigi
  min_period_sec: 0.02   # [s] minimum publish araligi
  max_period_sec: 0.5    # [s] maksimum publish araligi (heartbeat)
```

---

## 8. LAUNCH DOSYASI YAPISI

### 8.1 Phased Comparison Launch

```
agent_control_pkg/launch/phased_comparison.launch.py
```

Bu dosya 6 farkli test fazini destekler:
1. BASELINE - Ruzgarsiz
2. STEADY_WIND - Sabit ruzgar
3. TURBULENCE - Von Karman turbulans
4. GUST - Ani ruzgar
5. COMBINED - Stokastik karisim
6. ENDURANCE - Uzun sureli test

### 8.2 Collision Mode Parametresi

```bash
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=5 \
    collision_mode:=avoidance   # disabled, pass_through, avoidance
```

---

## 9. YENI OZELLIK EKLERKEN DIKKAT EDILECEKLER

### 9.1 Genel Kurallar

1. **Mevcut kontrolcu kiyaslamasini bozma** - PD/PID/IT2/GT2 karsilastirmasi temiz kalmali
2. **Opsiyonel yap** - Yeni ozellik `enable: false` default olmali
3. **Metrik topla** - Yeni ozelligin etkisini olcumle
4. **Backward compatible** - Eski launch dosyalari calismaya devam etmeli

### 9.2 Message Type Ekleme

```bash
# my_custom_interfaces_pkg/msg/ altina yeni .msg dosyasi ekle
# CMakeLists.txt'e ekle
# package.xml'e rosidl bagimliligi ekle
colcon build --packages-select my_custom_interfaces_pkg
```

### 9.3 Test Etme

```bash
# Build
colcon build --symlink-install --packages-select agent_control_pkg formation_coordinator_pkg

# Kisa test (60s)
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=false

# Collision avoidance ile test
ros2 launch agent_control_pkg phased_comparison.launch.py phase:=5 collision_mode:=avoidance
```

---

## 10. ONERILEN IMPLEMENTASYON: NEIGHBOR INFO PUBLISH

### 10.1 Adim 1: Yeni Message Type

```
# my_custom_interfaces_pkg/msg/NeighborPosition.msg
string agent_id
float64 x
float64 y
float64 z
float64 distance

# my_custom_interfaces_pkg/msg/NeighborInfo.msg
std_msgs/Header header
string agent_id
my_custom_interfaces_pkg/NeighborPosition[] neighbors
uint32 num_neighbors
```

### 10.2 Adim 2: Formation Coordinator'a Odom Subscribe

```cpp
// formation_coordinator_node.hpp - Eklenecekler

#include "nav_msgs/msg/odometry.hpp"
#include "my_custom_interfaces_pkg/msg/neighbor_info.hpp"

private:
  // Neighbor awareness icin
  bool neighbor_info_enable_{false};
  double neighbor_distance_threshold_{5.0};  // [m] bu mesafeden yakın olanlar "neighbor"

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Publisher<my_custom_interfaces_pkg::msg::NeighborInfo>::SharedPtr> neighbor_pubs_;

  struct AgentOdomState {
    double x{0.0}, y{0.0}, z{0.0};
    bool has_data{false};
    rclcpp::Time last_update;
  };
  std::unordered_map<std::string, AgentOdomState> agent_odom_states_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& agent_id);
  void publishNeighborInfo();
```

### 10.3 Adim 3: Neighbor Info Hesaplama ve Publish

```cpp
// formation_coordinator_node.cpp - Eklenecekler

void FormationCoordinatorNode::publishNeighborInfo()
{
  if (!neighbor_info_enable_) return;

  for (const auto& [agent_id, state] : agent_odom_states_) {
    auto msg = my_custom_interfaces_pkg::msg::NeighborInfo();
    msg.header.stamp = now();
    msg.agent_id = agent_id;

    for (const auto& [other_id, other_state] : agent_odom_states_) {
      if (other_id == agent_id) continue;

      double dx = other_state.x - state.x;
      double dy = other_state.y - state.y;
      double dz = other_state.z - state.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

      if (dist < neighbor_distance_threshold_) {
        auto neighbor = my_custom_interfaces_pkg::msg::NeighborPosition();
        neighbor.agent_id = other_id;
        neighbor.x = other_state.x;
        neighbor.y = other_state.y;
        neighbor.z = other_state.z;
        neighbor.distance = dist;
        msg.neighbors.push_back(neighbor);
      }
    }
    msg.num_neighbors = msg.neighbors.size();

    // Publish to /{agent_id}/neighbor_info
    auto pub_it = std::find_if(...);
    if (pub_it != neighbor_pubs_.end()) {
      (*pub_it)->publish(msg);
    }
  }
}
```

### 10.4 Adim 4: YAML Parametreleri

```yaml
# formation_12drone_it2_group.yaml - Eklenecekler

formation_coordinator_node:
  ros__parameters:
    # Neighbor awareness
    neighbor_info:
      enable: true
      distance_threshold: 5.0  # [m]
      publish_rate_hz: 10.0
```

---

## 11. TEZ ICIN SAVUNULABILIR IFADE

"Bu calismada cok ajanli formasyon problemi, merkezi bir koordinator tarafindan uretilen referanslar ve her ajan uzerinde calisan yerel kontrolculer (PD/PID/IT2/GT2) ile ele alinmistir. Carpizma guvenlik katmani, ajanlar arasi mesafe izleme ve APF tabanli hedef modifikasyonu ile cok-ajanli etkilesimi dogrudan olcen metrikler (min_distance, near_miss_count) uretmektedir. Bu metrikler, sistemin cok-ajanli ortamda guvenli calisma kapasitesini kanitlamaktadir."

---

## 12. DOSYA REFERANSLARI

| Dosya | Satir | Aciklama |
|-------|-------|----------|
| formation_coordinator_node.cpp | 287-299 | Publisher olusturma |
| formation_coordinator_node.cpp | 360-432 | Timer callback (target publish) |
| formation_coordinator_node.cpp | 677-766 | ETC event trigger |
| agent_controller_node.cpp | 46-56 | Subscription kurulum |
| agent_controller_node.cpp | 576-583 | Target callback |
| agent_controller_node.cpp | 585-592 | Odom callback |
| collision_safety_layer_node.cpp | 129-169 | Multi-agent subscription |
| collision_safety_layer_node.cpp | 211-253 | APF repulsion hesaplama |
| phased_comparison.launch.py | 304-341 | Agent node olusturma |
| phased_comparison.launch.py | 376-392 | Collision safety layer |

---

Bu dokuman, sistemin mevcut mimarisini ve "neighbor awareness" ozelligi eklemek icin gereken degisiklikleri detayli olarak aciklamaktadir.
