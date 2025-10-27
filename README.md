# 🤖 IoT Smart Mobile Robot System — Milestone 2  

---

## 🧭 Overview  

This project implements an **autonomous IoT-connected robot car** using a **Raspberry Pi 4B**.  
The system follows a line, detects and avoids obstacles, and streams live telemetry to **Adafruit IO** for remote monitoring and control.  
It fulfills the official **Milestone 2** requirements for the *IoT Smart Mobile Robot System* track.

**Key Features**
- Line-following with PID or rule-based control  
- Obstacle detection + safe stop/bypass using ultrasonic sensor  
- Remote start/stop + mode control via MQTT dashboard  
- Live telemetry: distance, motor speed, battery voltage, sensor states  
- Daily local data logging + auto cloud upload  
- Secure `.env` configuration (no hard-coded secrets)

---

## 🧩 System Architecture  

```text
┌──────────────────────────────┐
│  Sensors (IR, Ultrasonic, DHT)│
└────────────┬─────────────────┘
             │
     Raspberry Pi 4 (Python)
             │
   ┌─────────┴──────────┐
   │ Motor Driver (TB6612FNG) │
   │ Servo / Buzzer / LED     │
   └─────────┬──────────┘
             │
     MQTT (paho-mqtt)
             │
   Adafruit IO Dashboard
             │
      User Remote Control
