# -*- coding: utf-8 -*-
import os, sys, time, json, signal
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

# â”€â”€â”€ Load .env in current folder â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
load_dotenv()
AIO_USER = os.getenv("AIO_USERNAME", "").strip()
AIO_KEY  = os.getenv("AIO_KEY", "").strip()
PREFIX   = os.getenv("AIO_PREFIX", "smartpath").strip()
if not AIO_USER or not AIO_KEY:
    print("ERROR: Missing AIO_USERNAME or AIO_KEY in .env")
    sys.exit(1)

# â”€â”€â”€ Load config.json (feeds + tuning) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
CFG_PATH = os.path.join(os.path.dirname(__file__), "config.json")
try:
    with open(CFG_PATH, "r") as f:
        CFG = json.load(f)
except Exception as e:
    print(f"ERROR: Cannot read {CFG_PATH}: {e}")
    sys.exit(1)

def feed(key):
    """Resolve a short feed key from config to full AIO path."""
    k = CFG["feeds"][key]
    # allow either short ("smartpath.robot.speed") or full; normalize to full
    if "/" in k:
        # treat as short; prefix with username/feeds/
        return f"{AIO_USER}/feeds/{k}"
    return f"{AIO_USER}/feeds/{PREFIX}.{k}"

FEED_STARTSTOP = feed("startstop")
FEED_MODE      = feed("mode")
FEED_SPEED     = feed("speed")
FEED_DISTANCE  = feed("distance")
FEED_LINE      = feed("line")
FEED_BATTERY   = feed("battery")

AVOID_THRESH   = int(CFG["avoid"].get("threshold_cm", 20))
AVOID_REV_MS   = int(CFG["avoid"].get("reverse_ms", 350))
AVOID_TURN_MS  = int(CFG["avoid"].get("turn_ms", 350))

LINE_BASE      = int(CFG["line"].get("base_speed", 35))
LINE_KP        = float(CFG["line"].get("kp", 1200))

# â”€â”€â”€ Freenove kit imports â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
try:
    from motor import Ordinary_Car
except Exception as e:
    print("ERROR: Could not import motor/Ordinary_Car. Run from Code/Server folder.")
    raise

# Ultrasonic (try two common module names)
sonar = None
def read_distance_cm():
    return -1
try:
    from Ultrasonic import Ultrasonic
    sonar = Ultrasonic()
    def read_distance_cm():
        try:
            return int(sonar.get_distance())
        except Exception:
            return -1
except ImportError:
    try:
        from HCSR04 import HCSR04
        sonar = HCSR04()
        def read_distance_cm():
            try:
                return int(sonar.get_distance())
            except Exception:
                return -1
    except ImportError:
        pass

# Line tracking (IR array)
lt = None
def read_line_state():
    return "Unknown"
try:
    from Line_Tracking import Line_Tracking
    lt = Line_Tracking()
    if hasattr(lt, "status"):
        # Common convention: -1 left, 0 center, 1 right
        def read_line_state():
            try:
                s = lt.status()
                return { -1:"Left", 0:"Center", 1:"Right" }.get(s, "Unknown")
            except Exception:
                return "Unknown"
    elif hasattr(lt, "read_digital"):
        def read_line_state():
            try:
                return str(lt.read_digital())  # e.g., (0,1,0)
            except Exception:
                return "Unknown"
except ImportError:
    pass

# Battery reading (stub unless your kit exposes ADC.py)
def read_battery_v():
    return 0.0
try:
    import ADC
    def read_battery_v():
        try:
            # EXAMPLE only; adjust scale if your kit docs provide it
            raw = ADC.recvADC(0)
            return float(raw)  # replace with conversion to volts if known
        except Exception:
            return 0.0
except ImportError:
    pass

# â”€â”€â”€ Helpers & state â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def pct_to_pwm(p):
    p = max(0, min(100, int(p)))
    return int(round(p * 4095 / 100))

current_mode = "manual"
running      = False
speed_pct    = 25

car = None
client = None

def safe_stop():
    try:
        if car:
            car.set_motor_model(0,0,0,0)
            time.sleep(0.05)
    except Exception:
        pass

def shutdown(_sig=None, _frm=None):
    safe_stop()
    try:
        if client: client.loop_stop()
    except Exception:
        pass
    try:
        if car: car.close()
    except Exception:
        pass
    sys.exit(0)

# â”€â”€â”€ MQTT callbacks â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def on_connect(c, u, flags, rc):
    print("Connected rc=", rc)
    for f in (FEED_STARTSTOP, FEED_MODE, FEED_SPEED):
        c.subscribe(f); print("Subscribed:", f)

def on_message(c, u, msg):
    global running, current_mode, speed_pct
    val = msg.payload.decode().strip()
    print(f"MSG {msg.topic} -> {val}")
    if msg.topic == FEED_STARTSTOP:
        running = (val.upper() == "ON")
        if not running: safe_stop()
    elif msg.topic == FEED_MODE:
        current_mode = val.lower()
    elif msg.topic == FEED_SPEED:
        try:    speed_pct = max(0, min(100, int(float(val))))
        except: pass

# â”€â”€â”€ Drive helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def drive_forward_pct(p):
    v = pct_to_pwm(p); car.set_motor_model(v, v, v, v)
def drive_backward_pct(p):
    v = pct_to_pwm(p); car.set_motor_model(-v,-v,-v,-v)
def turn_left_pct(p):
    v = pct_to_pwm(p); car.set_motor_model(-v,-v, v, v)
def turn_right_pct(p):
    v = pct_to_pwm(p); car.set_motor_model( v, v,-v,-v)

def drive_manual():
    if not running: safe_stop(); return
    drive_forward_pct(max(20, speed_pct))

def drive_avoid():
    if not running: safe_stop(); return
    dist = read_distance_cm()
    if dist < 0:    safe_stop(); return
    if dist >= AVOID_THRESH:
        drive_forward_pct(max(20, speed_pct)); return
    # obstacle
    safe_stop(); time.sleep(0.1)
    drive_backward_pct(40); time.sleep(AVOID_REV_MS/1000.0)
    safe_stop(); time.sleep(0.1)
    (turn_left_pct if int(time.time())%2==0 else turn_right_pct)(45)
    time.sleep(AVOID_TURN_MS/1000.0)
    safe_stop()

# Simple scaffold for line mode (keeps safe stop until you tune it)
def drive_line():
    # Option 1: stop until you tune
    safe_stop()
    # Option 2 (enable after track ready):
    # state = read_line_state()
    # spd = max(LINE_BASE, speed_pct)
    # if state == "Left":   car.set_motor_model(-600,-600, pct_to_pwm(spd), pct_to_pwm(spd))
    # elif state == "Right":car.set_motor_model(pct_to_pwm(spd), pct_to_pwm(spd), -600,-600)
    # else:                 car.set_motor_model(pct_to_pwm(spd), pct_to_pwm(spd), pct_to_pwm(spd), pct_to_pwm(spd))

# â”€â”€â”€ Sensor publishing â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
_last_pub = 0.0
def publish_sensors(now):
    global _last_pub
    if now - _last_pub < 2.0: return
    _last_pub = now
    dist = read_distance_cm()
    line = read_line_state()
    batt = read_battery_v()
    client.publish(FEED_DISTANCE, dist)
    client.publish(FEED_LINE, line)
    client.publish(FEED_BATTERY, f"{batt:.2f}")
    print(f"Published â†’ dist:{dist} line:{line} batt:{batt:.2f}")

def main_loop():
    while True:
        if   current_mode == "manual": drive_manual()
        elif current_mode == "avoid" : drive_avoid()
        elif current_mode == "line"  : drive_line()
        else:                          safe_stop()
        publish_sensors(time.time())
        time.sleep(0.05)

# â”€â”€â”€ Entry â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
if __name__ == "__main__":
    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    car = Ordinary_Car()

    client = mqtt.Client()
    client.username_pw_set(AIO_USER, AIO_KEY)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("io.adafruit.com", 1883, 60)
    client.loop_start()

    try:
        main_loop()
    finally:
        shutdown()
