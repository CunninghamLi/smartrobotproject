# -*- coding: utf-8 -*-
# mqtt_drive.py
# Robot brain for Adafruit IO control + real sensors + manual, line, obstacle modes

import os, sys, time, json, signal, threading, collections, atexit, math
from pathlib import Path
from datetime import datetime, timezone
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

# === .env ===
load_dotenv()
AIO_USER = os.getenv("AIO_USERNAME", "").strip()
AIO_KEY  = os.getenv("AIO_KEY", "").strip()
PREFIX   = os.getenv("AIO_PREFIX", "smartpath").strip()
USE_TLS  = os.getenv("AIO_TLS", "1").strip().lower() in {"1","true","yes","on"}
if not AIO_USER or not AIO_KEY:
    print("ERROR: Missing AIO_USERNAME or AIO_KEY in .env")
    sys.exit(1)

# === paths ===
ROOT_DIR = Path(__file__).resolve().parents[0]
DATA_DIR = ROOT_DIR / "data"
LOGS_DIR = ROOT_DIR / "logs"
for d in (DATA_DIR, LOGS_DIR):
    d.mkdir(exist_ok=True)

# === config.json ===
CFG_PATH = ROOT_DIR / "config.json"
with CFG_PATH.open("r", encoding="utf-8") as f:
    CFG = json.load(f)

def feed(key: str) -> str:
    k = CFG["feeds"][key].strip()
    if k.startswith(f"{AIO_USER}/feeds/"):
        return k
    if "-dot-" in k:
        return f"{AIO_USER}/feeds/{k}"
    name = k if k.startswith(f"{PREFIX}.") else f"{PREFIX}.{k}"
    return f"{AIO_USER}/feeds/{name}"

def _full(topic_key: str) -> str:
    return f"{AIO_USER}/feeds/{topic_key}"

def _topic_variants(full_topic_with_prefix: str):
    try:
        _, key = full_topic_with_prefix.split("/feeds/", 1)
    except ValueError:
        return [full_topic_with_prefix]
    def both(k: str):
        return {k.replace(".", "-dot-"), k.replace("-dot-", ".")}
    cand = set()
    cand |= {_full(k) for k in both(key)}
    if key.startswith(f"{PREFIX}."):
        bare = key[len(f"{PREFIX}."):]
        cand |= {_full(k) for k in both(bare)}
    if key.startswith(f"{PREFIX}-dot-"):
        bare = key[len(f"{PREFIX}-dot-"):]
        cand |= {_full(k) for k in both(bare)}
    return list(cand)

def emergency_topic_variants(full_topic_with_prefix: str):
    return _topic_variants(full_topic_with_prefix)

# === Feeds ===
FEED_STARTSTOP = feed("startstop")
FEED_SPEED     = feed("speed")
FEED_EMERGENCY = feed("emergency")
EMERGENCY_TOPICS = emergency_topic_variants(FEED_EMERGENCY)
FEED_MODE      = feed("mode")

FEED_DISTANCE  = feed("distance")
FEED_LINE      = feed("line")
FEED_CAMERA    = feed("camera") if "camera" in CFG.get("feeds", {}) else None
FEED_MOTOR     = feed("motor")

FEED_LED     = feed("led")
FEED_SERVO0  = feed("servo0")
FEED_BUZZER  = feed("buzzer")

# === Freenove devices ===
from motor import Ordinary_Car
from infrared import Infrared
from ultrasonic import Ultrasonic
from led import Led
from servo import Servo
from buzzer import Buzzer

# === globals/state ===
STOP_EVENT      = threading.Event()
running         = False
emergency_on    = False
speed_pct       = 35
FORWARD_SIGN    = -1
current_mode    = "manual"

EMERGENCY_AUTO_RESUME = os.getenv("EMERGENCY_AUTO_RESUME", "0").strip().lower() in {"1","true","on","yes"}
_was_running_before_emergency = False

car             = None
client          = None
_pub_thread     = None

# Instantiate device objects
try:
    led_obj = Led()
except Exception as e:
    led_obj = None
    print("[init] LED init error:", e)

try:
    servo_obj = Servo()
except Exception as e:
    servo_obj = None
    print("[init] Servo init error:", e)

try:
    buzzer_obj = Buzzer()
except Exception as e:
    buzzer_obj = None
    print("[init] Buzzer init error:", e)

try:
    ir_sensor = Infrared()
except Exception as e:
    ir_sensor = None
    print("[init] Infrared init error:", e)

try:
    ultra_sensor = Ultrasonic()
except Exception as e:
    ultra_sensor = None
    print("[init] Ultrasonic init error:", e)

_last_pub_time_motor = 0.0
_last_pub_motor = {"combined": None}

_shutting_down = False
_car_closed = False

# sensor state + freshness
last_line      = "CENTER"
last_bits      = 7
last_distance  = None
t_line         = 0.0
t_distance     = 0.0
STALE_SEC      = 2.0
DIST_STOP_CM   = 12
REV_MS         = int(CFG.get("avoid", {}).get("reverse_ms", 350))

# === logging helpers (UTC) ===
USE_UTC = True
_log_last_write = 0.0
_log_cur_path = None
_log_header_written = False
_last_logged_motor_left = None
_last_logged_motor_right = None

def iso_now():
    return datetime.now(timezone.utc).isoformat() if USE_UTC else datetime.now().astimezone().isoformat()

def today_stamp():
    return datetime.now(timezone.utc).strftime("%Y-%m-%d") if USE_UTC else datetime.now().astimezone().strftime("%Y-%m-%d")

def telemetry_path():
    return DATA_DIR / f"{today_stamp()}_robot_telemetry.csv"

def events_path():
    return LOGS_DIR / f"{today_stamp()}_events.jsonl"

def ensure_csv_header(path: Path):
    global _log_header_written
    if path.exists() and path.stat().st_size > 0:
        _log_header_written = True
        return
    with path.open("w", encoding="utf-8") as f:
        f.write("timestamp,sensor_distance_cm,sensor_line_state,"
                "running,emergency,speed_pct,motor_left_pct,motor_right_pct,event\n")
    _log_header_written = True

def append_csv_row(path: Path, row: str):
    def _write():
        with path.open("a", encoding="utf-8") as f:
            f.write(row + "\n")
    _retry(_write, attempts=3, delay=0.3)

def log_event(msg: str):
    p = events_path()
    payload = {
        "timestamp": iso_now(),
        "event": msg,
        "running": running,
        "emergency": emergency_on,
        "speed_pct": speed_pct
    }
    def _append_jsonl():
        with p.open("a", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
            f.write("\n")
    _retry(_append_jsonl, attempts=3, delay=0.3)

# === MQTT publish queue ===
RATE_LIMIT_SECONDS = 2.1
_publish_lock = threading.Lock()
_publish_queue = collections.OrderedDict()
_last_pub_time = 0.0

def enqueue_publish(topic, payload, retain=False, qos=0):
    if not topic:
        return
    with _publish_lock:
        _publish_queue[topic] = (str(payload), retain, qos)

def flush_publish_queue_now():
    global _last_pub_time
    now = time.time()
    with _publish_lock:
        if not _publish_queue:
            return
        if now - _last_pub_time < RATE_LIMIT_SECONDS:
            return
        topic, (payload, retain, qos) = _publish_queue.popitem(last=True)
    try:
        if client:
            def _do_pub():
                return client.publish(topic, payload, qos=qos, retain=retain)
            _retry(_do_pub, attempts=3, delay=0.5)
            _last_pub_time = now
    except Exception as e:
        print("[pub] error:", e)

def publisher_loop():
    while not STOP_EVENT.is_set():
        flush_publish_queue_now()
        STOP_EVENT.wait(0.1)

def start_publisher_thread():
    global _pub_thread
    _pub_thread = threading.Thread(target=publisher_loop, daemon=True)
    _pub_thread.start()

# === helpers ===
def pct_to_pwm(p: int) -> int:
    p = max(0, min(100, int(p)))
    return int(round(p * 4095 / 100))

MIN_PWM_MOVE = 900

def scale_pwm(base: int) -> int:
    s = max(0, min(100, int(speed_pct))) / 100.0
    factor = 0.4 + 0.6 * s

    sign = 1 if base >= 0 else -1
    mag = abs(base)
    out = int(mag * factor)

    if mag > 0 and out < MIN_PWM_MOVE:
        out = MIN_PWM_MOVE

    return sign * out

def safe_stop():
    global _car_closed
    try:
        if car and not _car_closed:
            car.set_motor_model(0,0,0,0)
            time.sleep(0.05)
    except OSError as e:
        if getattr(e, "errno", None) != 9:
            print("[safe_stop] err:", e)
    except Exception as e:
        print("[safe_stop] err:", e)

def _retry(fn, *args, attempts=3, delay=0.5, **kwargs):
    for i in range(attempts):
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            print(f"[retry {i+1}/{attempts}] {getattr(fn, '__name__', 'call')} failed: {e}")
            time.sleep(delay)
    print("[retry] giving up")
    return None

def _is_on(v: str) -> bool:
    v = (v or "").strip().lower()
    return v in {"on","1","true","start","go","enabled","enable","yes","active"}

# === real sensors ===
# Start conservative. Change these if your debug shows mismatched patterns.
INVERT_LINE_BITS = False
SWAP_LR_BITS = False

def read_distance_cm():
    if ultra_sensor is None:
        return None
    try:
        d = ultra_sensor.get_distance()
        if d is None:
            return None
        d = float(d)
        if d <= 0:
            return None
        return d
    except Exception:
        return None

def read_line_bits_raw():
    if ir_sensor is None:
        return 7
    try:
        return ir_sensor.read_all_infrared() & 0x07
    except Exception:
        return 7

def normalize_line_bits(bits_raw: int):
    L = (bits_raw >> 2) & 1
    M = (bits_raw >> 1) & 1
    R = bits_raw & 1

    if INVERT_LINE_BITS:
        L = 1 - L
        M = 1 - M
        R = 1 - R

    if SWAP_LR_BITS:
        L, R = R, L

    return (L << 2) | (M << 1) | R

def read_line_state_from_bits(bits_norm: int):
    if bits_norm == 2:
        return "CENTER"
    if bits_norm in (4, 6):
        return "LEFT"
    if bits_norm in (1, 3):
        return "RIGHT"
    return "LOST"

def read_camera_status():
    return "online"

def read_camera_fps():
    return 12 + int(3 * math.sin(time.time()/3.0))

SENSOR_INTERVAL = 2.0
_last_sensor_pub_all = 0.0

def publish_sensors(now):
    global _last_sensor_pub_all
    if now - _last_sensor_pub_all < SENSOR_INTERVAL:
        return
    _last_sensor_pub_all = now

    try:
        d = read_distance_cm()
        if d is not None:
            enqueue_publish(FEED_DISTANCE, round(d, 2))
    except:
        pass

    try:
        enqueue_publish(FEED_LINE, last_line)
    except:
        pass

    if FEED_CAMERA:
        try:
            cam_payload = f"status={read_camera_status()},fps={read_camera_fps()}"
            enqueue_publish(FEED_CAMERA, cam_payload)
        except:
            pass

# === LED helpers ===
def leds_off():
    if led_obj is None:
        return
    try:
        led_obj.colorBlink(0)
        led_obj.ledIndex(0x00, 0, 0, 0)
    except Exception as e:
        print("[led] off error:", e)

def leds_color(r, g, b):
    if led_obj is None:
        return
    try:
        led_obj.ledIndex(0xFF, int(r), int(g), int(b))
    except Exception as e:
        print("[led] color error:", e)

def handle_led_command(val: str):
    v = val.strip().lower()
    if v in {"off", "0"}:
        leds_off()
    elif v in {"on", "white", "1"}:
        leds_color(255, 255, 255)
    elif v == "red":
        leds_color(255, 0, 0)
    elif v == "green":
        leds_color(0, 255, 0)
    elif v == "blue":
        leds_color(0, 0, 255)
    else:
        print("[led] unknown command:", val)

# === Servo helpers ===
def handle_servo0_command(val: str):
    if servo_obj is None:
        return
    v = val.strip().lower()
    try:
        if v in {"left", "0"}:
            servo_obj.set_servo_pwm("0", 0)
        elif v in {"center", "mid", "90"}:
            servo_obj.set_servo_pwm("0", 90)
        elif v in {"right", "180"}:
            servo_obj.set_servo_pwm("0", 180)
        else:
            angle = int(float(v))
            angle = max(0, min(180, angle))
            servo_obj.set_servo_pwm("0", angle)
    except Exception as e:
        print("[servo0] error:", e)

# === Buzzer helpers ===
def buzzer_off():
    if buzzer_obj is None:
        return
    try:
        buzzer_obj.set_state(False)
    except Exception as e:
        print("[buzzer] off error:", e)

def buzzer_on():
    if buzzer_obj is None:
        return
    try:
        buzzer_obj.set_state(True)
    except Exception as e:
        print("[buzzer] on error:", e)

def handle_buzzer_command(val: str):
    v = val.strip().lower()
    if v in {"off", "0"}:
        buzzer_off()
    elif v in {"on", "1"}:
        buzzer_on()
    elif v == "beep":
        buzzer_on()
        time.sleep(0.1)
        buzzer_off()
    else:
        print("[buzzer] unknown command:", val)

# === MQTT callbacks ===
def on_connect(c, u, flags, rc):
    print("Connected rc=", rc)
    for f in (FEED_STARTSTOP, FEED_SPEED, FEED_MODE, FEED_LED, FEED_SERVO0, FEED_BUZZER):
        c.subscribe(f)
        print("Subscribed:", f)
    for t in EMERGENCY_TOPICS:
        c.subscribe(t)
        print("Subscribed:", t)
    throttle_topic = f"{AIO_USER}/throttle"
    c.subscribe(throttle_topic)
    print("Subscribed:", throttle_topic)

def on_disconnect(c, u, rc):
    print("Disconnected rc=", rc)

def on_message(c, u, msg):
    global running, speed_pct, emergency_on, _was_running_before_emergency, current_mode

    val = msg.payload.decode(errors="ignore").strip()
    print(f"MSG {msg.topic} -> {val}")

    if msg.topic.endswith("/throttle"):
        print(f"[THROTTLE] {val}")
        return

    if msg.topic == FEED_STARTSTOP:
        req = _is_on(val)
        running = (req and not emergency_on)
        print(f"[startstop] requested={req} running={running} emergency={emergency_on}")
        if not running:
            safe_stop()

    elif msg.topic in EMERGENCY_TOPICS:
        prev = emergency_on
        emergency_on = _is_on(val)
        print(f"[emergency] {prev} -> {emergency_on}  (topic={msg.topic})")
        if emergency_on:
            _was_running_before_emergency = running
            running = False
            safe_stop()
            log_event("emergency_on")
        else:
            log_event("emergency_off")
            if EMERGENCY_AUTO_RESUME and _was_running_before_emergency:
                running = True

    elif msg.topic == FEED_SPEED:
        try:
            s_raw = float(val)
            s_clamped = max(0, min(100, int(round(s_raw))))
            speed_pct = s_clamped
        except Exception as e:
            print(f"[speed] invalid '{val}' ({e})")

    elif msg.topic == FEED_MODE:
        v = val.lower()
        if v.startswith("line"):
            current_mode = "line"
        elif v.startswith("obstacle"):
            current_mode = "obstacle"
        else:
            current_mode = "manual"
        print(f"[mode] -> {current_mode}")

    elif msg.topic == FEED_LED:
        handle_led_command(val)

    elif msg.topic == FEED_SERVO0:
        handle_servo0_command(val)

    elif msg.topic == FEED_BUZZER:
        handle_buzzer_command(val)

# === motors ===
def _apply_motor(a,b,c,d):
    """Used for manual and obstacle, applies FORWARD_SIGN."""
    a*=FORWARD_SIGN; b*=FORWARD_SIGN; c*=FORWARD_SIGN; d*=FORWARD_SIGN
    try:
        car.set_motor_model(a,b,c,d)
    except Exception as e:
        print("[motor] error:", e)

def _apply_motor_raw(a,b,c,d):
    """Used for Freenove tuned line values. No extra sign flip."""
    try:
        car.set_motor_model(a,b,c,d)
    except Exception as e:
        print("[motor] error:", e)

def _apply_line(a, b, c, d):
    # apply your car's forward orientation once for line mode
    s = FORWARD_SIGN
    _apply_motor_raw(s*a, s*b, s*c, s*d)


def _apply_motor_with_sign(sign, a,b,c,d):
    try:
        car.set_motor_model(sign*a, sign*b, sign*c, sign*d)
    except Exception as e:
        print("[motor] error:", e)

def _maybe_publish_motor_duty(a,b,c,d):
    global _last_pub_time_motor, _last_logged_motor_left, _last_logged_motor_right, _last_pub_motor
    now = time.time()
    if now - _last_pub_time_motor < 3.0:
        return
    _last_pub_time_motor = now
    try:
        to_pct = lambda v: int(round(abs(v)*100/4095))
        left  = to_pct((a+b)//2)
        right = to_pct((c+d)//2)
        _last_logged_motor_left = left
        _last_logged_motor_right = right
        combined = f"L={left},R={right}"
        if _last_pub_motor["combined"] != combined:
            enqueue_publish(FEED_MOTOR, combined)
            _last_pub_motor["combined"] = combined
    except Exception as e:
        print("[motor_pub] err:", e)

def drive_forward_pct(p):
    v = pct_to_pwm(p if (running and not emergency_on) else 0)
    _apply_motor(v,v,v,v)
    _maybe_publish_motor_duty(v,v,v,v)

def drive_backward_pct(p):
    v = pct_to_pwm(p if (running and not emergency_on) else 0)
    _apply_motor_with_sign(-FORWARD_SIGN, v, v, v, v)
    _maybe_publish_motor_duty(-v, -v, -v, -v)

def turn_left_pct(p):
    v = pct_to_pwm(p if (running and not emergency_on) else 0)
    _apply_motor(int(0.6*v), int(0.6*v), v, v)
    _maybe_publish_motor_duty(int(0.6*v), int(0.6*v), v, v)

def turn_right_pct(p):
    v = pct_to_pwm(p if (running and not emergency_on) else 0)
    _apply_motor(v, v, int(0.6*v), int(0.6*v))
    _maybe_publish_motor_duty(v, v, int(0.6*v), int(0.6*v))

# === behavior modes ===
def drive_manual():
    if not running or emergency_on:
        safe_stop()
        return
    drive_forward_pct(speed_pct)

def drive_line():
    if not running or emergency_on:
        safe_stop()
        return

    now = time.time()
    if now - t_line > STALE_SEC:
        safe_stop()
        return

    bits = last_bits

    if bits == 2:
        v = scale_pwm(800)
        _apply_line(v, v, v, v)
        _maybe_publish_motor_duty(v, v, v, v)

    elif bits == 4:
        a = scale_pwm(-1500)
        b = scale_pwm(2500)
        _apply_line(a, a, b, b)
        _maybe_publish_motor_duty(a, a, b, b)

    elif bits == 6:
        a = scale_pwm(-2000)
        b = scale_pwm(4000)
        _apply_line(a, a, b, b)
        _maybe_publish_motor_duty(a, a, b, b)

    elif bits == 1:
        a = scale_pwm(2500)
        b = scale_pwm(-1500)
        _apply_line(a, a, b, b)
        _maybe_publish_motor_duty(a, a, b, b)

    elif bits == 3:
        a = scale_pwm(4000)
        b = scale_pwm(-2000)
        _apply_line(a, a, b, b)
        _maybe_publish_motor_duty(a, a, b, b)

    else:
        safe_stop()

def drive_obstacle():
    if not running or emergency_on:
        safe_stop()
        return

    now = time.time()
    if last_distance is None or (now - t_distance) > STALE_SEC:
        safe_stop()
        return

    sp = max(0, speed_pct)

    if last_distance <= DIST_STOP_CM:
        safe_stop()
        time.sleep(0.1)

        drive_backward_pct(sp if sp > 0 else 25)
        time.sleep(REV_MS / 1000.0)

        safe_stop()
        time.sleep(0.1)

        turn_right_pct(max(25, sp))
        time.sleep(0.35)

        safe_stop()
    else:
        drive_forward_pct(sp)

# === main loop & shutdown ===
def main_loop():
    global _log_last_write, _log_cur_path, _log_header_written
    global last_line, last_distance, t_line, t_distance, last_bits

    while not STOP_EVENT.is_set():
        now = time.time()

        try:
            bits_raw = read_line_bits_raw()
            last_bits = normalize_line_bits(bits_raw)
            last_line = read_line_state_from_bits(last_bits)
            t_line = now
        except:
            pass

        try:
            d = read_distance_cm()
            if d is not None:
                last_distance = d
                t_distance = now
        except:
            pass

        if current_mode == "manual":
            drive_manual()
        elif current_mode == "line":
            drive_line()
        elif current_mode == "obstacle":
            drive_obstacle()
        else:
            safe_stop()

        if now - _log_last_write >= 2.0:
            _log_last_write = now
            path = telemetry_path()
            if _log_cur_path != path:
                _log_cur_path = path
                _log_header_written = False
            if not _log_header_written:
                ensure_csv_header(path)

            ts   = iso_now()
            dist = last_distance if last_distance is not None else ""
            line = last_line
            ml = "" if _last_logged_motor_left  is None else _last_logged_motor_left
            mr = "" if _last_logged_motor_right is None else _last_logged_motor_right
            row = f'{ts},{dist},{line},{int(running)},{int(emergency_on)},{speed_pct},{ml},{mr},'
            append_csv_row(path, row)

        publish_sensors(now)
        STOP_EVENT.wait(0.05)

def _shutdown_sequence():
    global _shutting_down, _car_closed
    if _shutting_down:
        return
    _shutting_down = True
    try:
        STOP_EVENT.set()
        safe_stop()
        buzzer_off()
        leds_off()
        try:
            if ir_sensor: ir_sensor.close()
        except:
            pass
        try:
            if ultra_sensor and hasattr(ultra_sensor, "close"): ultra_sensor.close()
        except:
            pass

        if client:
            try:
                flush_publish_queue_now()
                client.loop_stop()
                client.disconnect()
            except Exception as e:
                print("[shutdown] mqtt:", e)

        if _pub_thread and _pub_thread.is_alive():
            _pub_thread.join(timeout=1.5)

        if car and not _car_closed:
            try:
                car.close()
                _car_closed = True
            except Exception as e:
                print("[shutdown] car:", e)

    except Exception as e:
        print("[shutdown] general:", e)

def _signal_handler(sig=None, frm=None):
    print("Caught signal, shutting down...")
    _shutdown_sequence()

signal.signal(signal.SIGINT,  _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
atexit.register(_shutdown_sequence)

# === entry ===
if __name__ == "__main__":
    try:
        car = Ordinary_Car()
        client = mqtt.Client()
        client.username_pw_set(AIO_USER, AIO_KEY)

        if USE_TLS:
            try:
                client.tls_set()
            except Exception as e:
                print("[mqtt] tls_set warning:", e)
            port = 8883
        else:
            port = 1883

        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message

        client.connect("io.adafruit.com", port, 60)
        client.loop_start()

        start_publisher_thread()
        main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_sequence()
        print("Bye.")

