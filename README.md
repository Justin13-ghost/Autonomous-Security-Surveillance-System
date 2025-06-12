# 🛡️ Autonomous Security Surveillance System

This is a two-part smart surveillance system that combines motion sensing, computer vision, and wireless communication using ESP32, Raspberry Pi 5, YOLOv8, OpenCV, and MQTT. The project features custom 3D-printed hardware, servo control, and real-time tracking of intruders or other target classes.

---

##  System Architecture

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

##  Demonstration

<!-- Insert images or demo GIFs -->
![Radar View](images/radar_demo.gif)
![Tracking View](images/tracking_demo.gif)

---

##  Features

-  Real-time 180° ultrasonic scanning radar
-  MQTT-based inter-device communication
-  YOLOv8 object detection on Raspberry Pi
-  Target tracking using custom pan-tilt servo algorithm
-  Touchscreen interface and Node-RED dashboard
-  Fully 3D-printed mechanical design

---

## 📁 Project Structure
project-root/
├── arduino-code/              # Arduino code for ESP32-based scanning turret
│   └── radar_scanner.ino
│
├── processing-ui/             # Processing sketch for radar-style display
│   └── radar_visualizer.pde
│
├── raspberry-pi-code/         # Python code for Raspberry Pi-based camera turret
│   ├── object_tracker.py      # Main YOLOv8 + OpenCV tracking script
│   ├── mqtt_listener.py       # Subscribes to ESP32 distance data
│   └── utils/                 # Helper modules
│       ├── servo_controller.py
│       └── yolov8_interface.py
│
├── nodered-flows/             # Node-RED flow export for live data display
│   └── nodered_flow.json
│
├── stl-files/                 # 3D printable parts for both turret systems
│   ├── esp32_servo_turret/
│   └── pi_pan_tilt_mount/
│
├── media/                     # Images and demo videos for README and documentation
│   ├── demo.gif
│   ├── system_diagram.png
│   └── screenshots/
│
├── LICENSE                    # MIT License (for code)
├── LICENSE-mechanical         # CC BY 4.0 License (for STL files and diagrams)
└── README.md                  # Project overview and documentation


---

##  Requirements

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

##  Setup Instructions

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

