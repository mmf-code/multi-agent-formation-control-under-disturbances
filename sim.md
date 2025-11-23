# Simülasyon + Dashboard Çalıştırma Rehberi

Bu dosya, **gerçek 9‑drone simülasyonu** ve **dashboard backend/frontend**’ini birlikte nasıl çalıştıracağını özetler. İki tip kullanım var:

- Gerçek simülasyon (9 drone, 3 grup, 3 farklı kontrolcü)
- Hafif test (mock/test publisher ile sadece dashboard’ı denemek)

> Not: Aşağıdaki komutlar repository kökünden (`multi-agent-formation-control-under-disturbances/`) çalıştırılacak şekilde yazıldı.

---

## 1. Ortam Hazırlığı (Her Zaman)

Her yeni terminalde önce ROS ve workspace’i source et:

```bash
cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 2. Dashboard Backend’i Başlatma

Backend, FastAPI + ROS bridge + WebSocket sunucusudur. Ayrı bir terminalde:

```bash
cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash

cd monitoring_dashboard/backend
python3 app.py
```

- HTTP API: `http://localhost:8000`
- WebSocket: `ws://localhost:8000/ws` (ve `ws://localhost:8000/ui/ws`)
- UI (build edilmişse): `http://localhost:8000/ui`

Backend bir kere açılır; gerçek sim veya test publisher bunu otomatik kullanır.

---

## 3. Gerçek 9 Drone Simülasyonu (3 Grup, 3 Kontrolcü)

Bu modda Gazebo + RViz + 9 drone + formasyon + gerçek kontrolcü parametreleri kullanılır.

Yeni bir terminalde:

```bash
cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash

# Görsel demo (9 drone, 3 grup)
./scripts/run_formation_demo.sh
```

Alternatif olarak doğrudan launch dosyası:

```bash
ros2 launch agent_control_pkg formation_comparison_demo.launch.py
```

Bu senaryoda:

- Grup 0 (agent_0,1,2): PID+Fuzzy
- Grup 1 (agent_3,4,5): PD
- Grup 2 (agent_6,7,8): PID

Dashboard tarafında beklenenler:

- `System Status` panelinde **Active Agents: 9**
- Topic sayısı ~36
- Haritada 9 drone pozisyonu
- Metrics / odom / wind / diagnostics grafikleri canlı güncellenir

> Önemli: Gerçek sim çalışırken **test publisher açık olmamalı** (aşağıya bak).

./scripts/run_formation_demo.sh --headless --scenario=phased

---

## 4. Test Publisher (Mock 3 Agent – Dashboard’ı Tek Başına Denemek)

Simülasyon açmadan, sadece dashboard UI’yi denemek için mock yayıncıyı kullanabilirsin.

Yeni bir terminalde:

```bash
cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash

python3 monitoring_dashboard/scripts/test_publisher.py
```

Bu node:

- `agent_0`, `agent_1`, `agent_2` için:
  - `/agent_X/odom`
  - `/agent_X/target_pose`
  - `/agent_X/diagnostics`
  - `/agent_X/controller_params`
- Global:
  - `/wind/velocity`
  - `/wind/force`

yayınlar. Dashboard’da:

- 3 agent görürsün
- Controller Parameters paneli **mock Kp/Ki/Kd** ile dolu olur

> Bu veriler **gerçek sim parametreleri değil**, sadece UI test içindir.

---

## 5. Mock → Gerçek Sim’e Geçerken Dikkat

cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_formation_demo.sh

ros2 launch agent_control_pkg formation_comparison_demo.launch.py



Gerçek 9’lu sim’e geçtiğinde, ROS graph’ın temiz olması için:

1. Test publisher’ı durdur:
   - Çalıştığı terminalde `Ctrl+C`  
   - veya:
     ```bash
     ros2 node list          # /dashboard_test_publisher görünüyor mu?
     ros2 node kill /dashboard_test_publisher
     ```

2. Gerekirse Gazebo’yu temizle:
   ```bash
   pkill -9 gzserver; pkill -9 gzclient
   ```

3. Backend’i yeniden başlat (özellikle mock’tan sonra):
   ```bash
   # Eski backend terminalinde Ctrl+C
   # Sonra:
   cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   cd monitoring_dashboard/backend
   python3 app.py
   ```

4. Ardından gerçek sim launch’ını çalıştır (bkz. Bölüm 3).

Bu sırayla gittiğinde backend sadece **gerçek** `/agent_0..8/...` topic’lerini görecek; 3 sahte agent kaybolacak.

---

## 6. Dashboard’ı Açma

Backend çalışıyorken tarayıcıdan:

```text
http://localhost:8000/ui
```

Beklenen:

- Sağ üstte:
  - **Connected** (yeşil) → WebSocket bağlı
- System Status:
  - Sim yoksa: `ROS2 Active`, `Active Agents (0)`
  - Test publisher ile: `Active Agents (3)`
  - Gerçek sim ile: `Active Agents (9)`

Buradan sonra dashboard’ı daha da geliştirmek için bu dosyayı genişletebiliriz (örneğin ek topic’ler, yeni grafikler, vs.).  

---

## 7. Build / Workspace Reset (Önemli Backup Notları)

Sim, dashboard veya paketler saçma hatalar vermeye başlarsa, önce **workspace’i tamamen temizleyip** tekrar build etmeyi dene.

### 7.1. Tam Temiz Build

```bash
cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances

# Her şeyi sıfırla
rm -rf build install log

# ROS2 ortamı
source /opt/ros/humble/setup.bash

# Tüm paketleri yeniden derle
colcon build --symlink-install
```

Build bittikten sonra **HER yeni terminalde**:

```bash
cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Paketi gerçekten görüyor mu diye kontrol:

```bash
ros2 pkg list | grep agent_control_pkg
```

Eğer burada `agent_control_pkg` görünüyorsa, sim scriptleri (`run_formation_demo.sh` vs.) o terminalde düzgün çalışır.

### 7.2. Sık Görülen Hatalar ve Çözümleri

- **Hata:**  
  `Package 'agent_control_pkg' not found: "package 'agent_control_pkg' not found, searching: ['/opt/ros/humble']"`  
  **Sebep:** O terminalde workspace’i (`install/setup.bash`) source etmedin veya build daha önce patlamış.  
  **Çözüm:**  
  ```bash
  cd ~/Documents/GitHub/multi-agent-formation-control-under-disturbances
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 pkg list | grep agent_control_pkg
  ```  
  Eğer listede yoksa 7.1’deki tam temiz build’i uygula.

- **Hata (eski, şu an repo’da fixli):**  
  CMake `my_custom_interfaces_pkg` paketini bulamıyor (`find_package(my_custom_interfaces_pkg)` hatası).  
  **Sebep:** `agent_control_pkg/package.xml` içinde dependency eksik olmasıydı; şu an repo’da:  
  ```xml
  <depend>my_custom_interfaces_pkg</depend>
  ```  
  satırı eklendi. Eğer benzer bir custom paket eklerken bu hatayı yeniden görürsen:
  - İlgili paketin `package.xml` ve `CMakeLists.txt` dosyalarında dependency’leri eklediğinden emin ol.
  - Ardından 7.1’deki tam temiz build adımlarını uygula.
