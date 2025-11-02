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

