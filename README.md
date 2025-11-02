# 🤖 Smart Raspberry Pi IoT Robot — Milestone 2

Champlain College St-Lambert
Internet of Things / Smart Robot Project  

---

## 👤 Team Members
| Name | Role |
|------|------|
| **Cunningham Li** | Full system implementation: Raspberry Pi setup, MQTT, dashboard, logging, wiring |

---

## 🧠 System Overview

This project implements a **smart IoT robot** using a Raspberry Pi and MQTT (Adafruit IO).  
The robot can be controlled remotely via a cloud dashboard and logs its telemetry data locally.

### ✅ Key Features
- Cloud control (Adafruit IO MQTT)
- Start / Stop motor control
- Adjustable speed (0–100%)
- Emergency stop override
- Local CSV logging & JSON event logging
- Daily file rotation
- Heartbeat status feed
- Graceful shutdown + retry logic

> ⚠️ Hardware Note: Freenove sensor board malfunctioned.  
Sensors are simulated, but all IoT logic, logging, MQTT feeds, and dashboard work fully.

---

## 📦 System Block Diagram

              🌐 Adafruit IO (Cloud Dashboard)
                      |
                      |  MQTT Commands + Status
                      v
        +--------------------------------------+
        |     Raspberry Pi (Python Program)    |
        |--------------------------------------|
        | • MQTT Client (paho-mqtt)            |
        | • Motor Control (PWM)                |
        | • Speed Control (0–100%)             |
        | • Emergency Stop Logic               |
        | • Heartbeat Feed                     |
        | • Local CSV & JSONL Logging          |
        | • Retry & Graceful Shutdown          |
        +--------------------+-----------------+
                             |
                             | GPIO Ribbon Cable
                             v
              +-------------------------------+
              |   Freenove Motor Driver PCB   |
              |   • H-Bridges for 4 Motors    |
              +-------------------------------+
                             |
                             | DC Power
                             v
              🚗 DC Motors (4-Wheel Drive Robot)

---

## 🧾 Bill of Materials (BOM)

| Component | Model / Part Number | Quantity | Link |
|----------|---------------------|---------|------|
| **Raspberry Pi 4 Model B (4GB)** | RPI4-4GB | 1 | https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ |
| **Freenove 4WD Smart Car Kit for Raspberry Pi** | FNK0042 (kit bundle) | 1 | https://www.amazon.ca/dp/B07YD2LT9D |
| **MicroSD Card 32GB (OS Storage)** | SanDisk Ultra 32GB | 1 | https://www.amazon.ca/dp/B07H4V6N65 |
| **Power Bank (5V USB Output)** | Generic portable battery | 1 | https://www.amazon.ca/dp/B08JRX7W9X |
| **USB-C Power Cable for Raspberry Pi** | Standard USB-C cable | 1 | https://www.amazon.ca/dp/B07Y8D67W2 |
| **Jumper Wires / Ribbon Cable** | Included in Freenove kit | — | Included in kit |
| **Screwdriver & Assembly Tools** | Included in Freenove kit | — | Included in kit |

> Note: Sensor board from kit malfunctioned — motors and cloud control still implemented successfully.

---

## 🔌 Wiring Diagram / Schematics and Photos

![IMG_9640](https://github.com/user-attachments/assets/10d2fea4-6d8a-4df1-8c01-2384a6f1783f)

---

## ⚙️ Setup Instructions (OS Prep, Dependencies, Environment Variables)

- Clone the repo
- Install python dependencies in requirements.txt
- Create .env and config.json

---

## ▶️ How to Run the Robot

- Change directory to robot_iot/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server
- Run python3 mqtt_drive.py

---

## 🗄️ Data Format Specification & File Rotation Policy

This project logs robot telemetry data locally on the Raspberry Pi to ensure reliability even if the network is unavailable. Two log files are created each day: one CSV file for telemetry and one JSON lines file for important events.

### Telemetry CSV format (logged every ~2 seconds):
- timestamp (ISO-8601, UTC)
- distance sensor reading (cm) – simulated (blank due to hardware issue)
- line sensor state – simulated (blank)
- battery voltage – simulated (blank)
- running state (0 or 1)
- emergency state (0 or 1)
- speed command (% from Adafruit dashboard)
- left motor duty cycle (%)
- right motor duty cycle (%)

### Event log format (JSONL, whenever an important event happens):
- timestamp
- event name (ex: “emergency_on”)
- robot state (running or stopped)
- speed value at event moment

### File rotation policy:
- Log files are created daily
- File name format: YYYY-MM-DD_robot_telemetry.csv and YYYY-MM-DD_events.jsonl
- Files are stored locally in data/ and logs/ folders
- Old log files remain saved for history unless manually deleted
- UTC timezone used for consistent timestamps

### Purpose:
- Ensures long-term data storage
- Protects against cloud outages
- Provides evidence of robot operation history for milestone requirements

---


  
