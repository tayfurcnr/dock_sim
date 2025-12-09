# Dock Station Control Panel

Modern SCADA arayüzü - ROS entegrasyonlu dock istasyonu kontrol paneli.

## 📁 Dosya Yapısı

```
user_interface/
├── config/              # Konfigürasyon dosyaları
│   └── config.yaml      # ROS topic/service ayarları
├── core/                # Çekirdek modüller
│   ├── data.py          # Sample telemetri verisi
│   └── ros_bridge.py    # ROS entegrasyon katmanı
├── ui/                  # UI bileşenleri
│   ├── pages/           # LCD ekran sayfaları
│   │   ├── summary_page.py
│   │   ├── weather_page.py
│   │   ├── location_page.py
│   │   ├── system_page.py
│   │   └── charger_page.py
│   ├── lcd_display.py   # LCD ekran widget'ı
│   └── widgets.py       # Mekanik kontrol widget'ları
├── app.py               # Ana uygulama
├── requirements.txt     # Python bağımlılıkları
└── README.md

```

## 🚀 Kurulum

### Standalone Kurulum

```bash
# Bağımlılıkları yükle
pip install -r requirements.txt
```

### ROS Paketi İçine Entegrasyon (Önerilen)

```bash
# Mevcut Gazebo ROS paketinin içine kopyala
cd ~/catkin_ws/src/your_gazebo_package/
mkdir -p scripts
cp -r ~/catkin_ws/dock_sim_ui scripts/

# run_ui.py'yi executable yap
chmod +x scripts/dock_sim_ui/run_ui.py

# CMakeLists.txt'ye ekle:
# catkin_install_python(PROGRAMS
#   scripts/dock_sim_ui/run_ui.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Paketi derle
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch File Örneği

```xml
<!-- your_gazebo_package/launch/dock_with_ui.launch -->
<launch>
  <!-- Gazebo simulation -->
  <include file="$(find your_gazebo_package)/launch/dock_simulation.launch"/>
  
  <!-- Dock UI -->
  <node name="dock_ui" pkg="your_gazebo_package" type="run_ui.py" output="screen"/>
</launch>
```

## ▶️ Çalıştırma

### Standalone Mode

```bash
# Demo mode (ROS olmadan)
python app.py

# ROS mode (config.yaml'da enabled: true olmalı)
python app.py
```

### ROS Launch ile

```bash
# UI ile birlikte tüm sistemi başlat
roslaunch your_gazebo_package dock_with_ui.launch

# Sadece UI'yi başlat (simulation zaten çalışıyorsa)
rosrun your_gazebo_package run_ui.py
```

## ⚙️ Konfigürasyon

`config/config.yaml` dosyasında:
- `ros.enabled`: ROS kullanımını aç/kapa
- `ros.topics`: ROS topic'leri tanımla
- `ros.services`: ROS service'leri tanımla
- `ros.button_actions`: Buton-service eşleştirmeleri

## 🎨 Özellikler

### Sol Panel (Mekanik Kontroller)
- 3 LED paneli (System, Speaker, Lid)
- Speaker grill widget
- 6 kontrol butonu (Lid Open/Close/Stop, Manual Open/Close, E-Stop)

### Sağ Panel (LCD Ekran)
- **HOME**: Lid durumu, çevre verileri, network durumu
- **WEATHER**: Hava durumu kartları (sıcaklık, nem, basınç, ışık, rüzgar, yağmur)
- **LOCATION**: Google Maps entegrasyonu
- **SYSTEM**: Sistem hata ve network bilgileri
- **CHARGER**: Şarj kanalı detayları

## 🔌 ROS Entegrasyonu

### Subscribe Edilen Topic'ler
- `/dock/lid/state` (Lid)
- `/dock/weather/env` (WeatherEnv)
- `/dock/weather/wind` (WeatherWind)
- `/dock/weather/rain` (WeatherRain)
- `/dock/location` (Location)
- `/dock/system/error` (SystemError)
- `/dock/system/network` (SystemNetwork)

### Kullanılan Service'ler
- `/dock/lid/trigger` (LidControlTrigger)
  - Command 0: CLOSE
  - Command 1: OPEN
  - Command 2: STOP

## 📝 Notlar

- Manual butonlar sadece lokal UI durumunu günceller, ROS'a mesaj göndermez
- E-Stop butonu tüm operasyonları durdurur
- Demo mode'da sample data ile çalışır
