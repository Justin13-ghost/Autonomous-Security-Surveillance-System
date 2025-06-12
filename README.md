# 🛡️ Autonomous Security Surveillance System

This is a two-part smart surveillance system that combines motion sensing, computer vision, and wireless communication using ESP32, Raspberry Pi 5, YOLOv8, OpenCV, and MQTT. The project features custom 3D-printed hardware, servo control, and real-time tracking of intruders or other target classes.

---

## 🔧 System Architecture

### **Part 1: Scanning Radar with ESP32**

- **Hardware:**
  - ESP32 microcontroller
  - HC-SR04 Ultrasonic Sensor
  - Servo motor (mounted in a custom 3D-printed turret)
  - USB serial connection for data visualization
- **Functionality:**
  - Continuously sweeps 180° using a servo
  - Detects distance via ultrasonic sensor
  - Sends angle + distance over MQTT
  - Real-time radar visualization via **Processing IDE**
- **Software:**
  - Arduino code for servo sweeping, ultrasonic reading, and MQTT publishing
  - Processing sketch for radar-like visual interface

### **Part 2: AI-Powered Pan-Tilt Camera with Raspberry Pi**

- **Hardware:**
  - Raspberry Pi 5
  - Pi Camera Module V3
  - Dual-servo pan-tilt mechanism (3D-printed)
  - HDMI touchscreen for local display
- **Functionality:**
  - Listens for MQTT messages from ESP32
  - When motion is detected:
    - Wakes up from idle state
    - Scans field of view using YOLOv8 for object detection
    - Tracks target (e.g., person) using OpenCV + custom servo logic
    - Returns to idle if nothing is detected
  - Object detection confidence + bounding box display
- **Software:**
  - Python code for MQTT, OpenCV, YOLOv8, and servo control
  - Node-RED dashboard to monitor angle and distance data in real time

---

## 📸 Demonstration

<!-- Insert images or demo GIFs -->
![Radar View](images/radar_demo.gif)
![Tracking View](images/tracking_demo.gif)

---

## 💡 Features

- 📡 Real-time 180° ultrasonic scanning radar
- 🌐 MQTT-based inter-device communication
- 🧠 YOLOv8 object detection on Raspberry Pi
- 🎯 Target tracking using custom pan-tilt servo algorithm
- 🖥️ Touchscreen interface and Node-RED dashboard
- 🔧 Fully 3D-printed mechanical design

---

## 📁 Project Structure
├── esp32/
│ ├── radar_scan.ino # Arduino code for ultrasonic sweep + MQTT
│
├── processing/
│ ├── radar_visualizer.pde # Processing radar interface
│
├── raspberry_pi/
│ ├── tracking_camera.py # Python script with YOLOv8 + OpenCV
│ ├── servo_control.py # Custom object-centering algorithm
│ ├── mqtt_subscriber.py # MQTT integration
│ └── node_red_flow.json # Node-RED dashboard flow
│
├── stl_files/ # 3D printable parts
│ ├── turret_mount.stl
│ ├── pan_base.stl
│ └── tilt_arm.stl
│
└── README.md


---

## 📦 Requirements

### ESP32:
- Arduino IDE
- PubSubClient (for MQTT)
- Servo.h library

### Raspberry Pi:
- Python 3.9+
- `opencv-python`
- `ultralytics` (YOLOv8)
- `paho-mqtt`
- `gpiozero` or `RPi.GPIO`
- Node-RED (optional)

---

## 🚀 Setup Instructions

### 1. ESP32
- Flash `radar_scan.ino` to your ESP32 via Arduino IDE
- Connect ultrasonic sensor and servo as described in the code
- Connect ESP32 to MQTT broker

### 2. Processing Radar Interface
- Open `radar_visualizer.pde` in Processing IDE
- Run it while ESP32 is connected over USB serial

### 3. Raspberry Pi 5
- Install Python dependencies:  
  ```bash
  pip install opencv-python paho-mqtt ultralytics

