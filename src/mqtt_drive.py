# -*- coding: utf-8 -*-
# mqtt_drive.py â€” movement only, throttle-safe, graceful Ctrl+C, Emergency Stop,
#                 + Local Data Logging (daily CSV rotation + events JSONL, UTC timestamps)
#                 + Basic retries for MQTT publish and file I/O

import os, sys, time, json, signal, threading, collections, atexit
from pathlib import Path
from datetime import datetime, timezone
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

# === .env ===
load_dotenv()
AIO_USER = os.getenv("AIO_USERNAME", "").strip()
AIO_KEY  = os.getenv("AIO_KEY", "").strip()
PREFIX   = os.getenv("AIO_PREFIX", "smartpath").strip()
if not AIO_USER or not AIO_KEY:
    print("ERROR: Missing AIO_USERNAME or AIO_KEY in .env")
    sys.exit(1)

# === paths (data/logs for local history) ===
ROOT_DIR = Path(__file__).resolve().parents[0]
DATA_DIR = ROOT_DIR / "data"         # CSV telemetry
LOGS_DIR = ROOT_DIR / "logs"         # JSONL events
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

def emergency_topic_variants(full_topic_with_prefix: str):
    # Builds with-prefix dotted/-dot- + no-prefix dotted/-dot-
    try:
        _, key = full_topic_with_prefix.split("/feeds/", 1)
    except ValueError:
        return [full_topic_with_prefix]

    def to_variants(k: str):
        if "-dot-" in k:
            return (k, k.replace("-dot-", "."))
        else:
            return (k.replace(".", "-dot-"), k)

    with_dot, with_dotted = to_variants(key)

    no_prefix_key = key
    if key.startswith(f"{PREFIX}-dot-"):
        no_prefix_key = key[len(f"{PREFIX}-dot-"):]
    elif key.startswith(f"{PREFIX}."):
        no_prefix_key = key[len(f"{PREFIX}."):]
    no_dot, no_dotted = to_variants(no_prefix_key)

    topics = {
        _full(with_dot),
        _full(with_dotted),
        _full(no_dot),
        _full(no_dotted),
    }
    return list(topics)

FEED_STARTSTOP = feed("startstop")
FEED_SPEED     = feed("speed")
FEED_EMERGENCY = feed("emergency")
EMERGENCY_TOPICS = emergency_topic_variants(FEED_EMERGENCY)
FEED_MOTOR_L   = feed("motor_l")
FEED_MOTOR_R   = feed("motor_r")
FEED_HEARTBEAT = feed("heartbeat")

# === Freenove motor ===
from motor import Ordinary_Car

# === globals/state ===
STOP_EVENT      = threading.Event()     # graceful shutdown flag
running         = False
emergency_on    = False
speed_pct       = 35
FORWARD_SIGN    = -1

EMERGENCY_AUTO_RESUME = os.getenv("EMERGENCY_AUTO_RESUME", "0").strip().lower() in {"1","true","on","yes"}
_was_running_before_emergency = False

car             = None
client          = None
_pub_thread     = None

_last_hb        = 0.0
_last_applied_speed = None
_last_pub_motor = {"l": None, "r": None}
_last_pub_time_motor = 0.0

# shutdown idempotence flags
_shutting_down = False
_car_closed = False

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
    # Example: data/2025-11-01_robot_telemetry.csv
    return DATA_DIR / f"{today_stamp()}_robot_telemetry.csv"

def events_path():
    # Example: logs/2025-11-01_events.jsonl
    return LOGS_DIR / f"{today_stamp()}_events.jsonl"

def ensure_csv_header(path: Path):
    global _log_header_written
    if path.exists() and path.stat().st_size > 0:
        _log_header_written = True
        return
    with path.open("w", encoding="utf-8") as f:
        f.write("timestamp,sensor_distance_cm,sensor_line_state,sensor_battery_v,"
                "running,emergency,speed_pct,motor_left_pct,motor_right_pct,event\n")
    _log_header_written = True

def append_csv_row(path: Path, row: str):
    # file I/O with basic retries
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
    # file I/O with basic retries
    def _append_jsonl():
        with p.open("a", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
            f.write("\n")
    _retry(_append_jsonl, attempts=3, delay=0.3)

# === MQTT publish rate limiter (coalescing) ===
RATE_LIMIT_SECONDS = 2.1   # ~30/min on free plan
_publish_lock = threading.Lock()
_publish_queue = collections.OrderedDict()
_last_pub_time = 0.0

def enqueue_publish(topic, payload, retain=False):
    with _publish_lock:
        _publish_queue[topic] = (str(payload), retain)

def flush_publish_queue_now():
    global _last_pub_time
    now = time.time()
    with _publish_lock:
        if not _publish_queue:
            return
        if now - _last_pub_time < RATE_LIMIT_SECONDS:
            return
        topic, (payload, retain) = _publish_queue.popitem(last=True)
    try:
        if client:
            # MQTT publish with basic retries (doesn't change queue/throttle behavior)
            def _do_pub():
                return client.publish(topic, payload, retain=retain)
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

# === core helpers ===
def pct_to_pwm(p: int) -> int:
    p = max(0, min(100, int(p)))
    return int(round(p * 4095 / 100))

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

# minimal generic retry helper (used above)
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

# === MQTT callbacks ===
def on_connect(c, u, flags, rc):
    print("Connected rc=", rc)
    for f in (FEED_STARTSTOP, FEED_SPEED):
        c.subscribe(f); print("Subscribed:", f)
    for t in EMERGENCY_TOPICS:
        c.subscribe(t); print("Subscribed:", t)
    throttle_topic = f"{AIO_USER}/throttle"
    c.subscribe(throttle_topic); print("Subscribed:", throttle_topic)
    enqueue_publish(FEED_HEARTBEAT, "online", retain=True)

def on_disconnect(c, u, rc):
    print("Disconnected rc=", rc)

def on_message(c, u, msg):
    global running, speed_pct, emergency_on, _was_running_before_emergency
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
            print("[emergency] STOP engaged")
        else:
            log_event("emergency_off")
            print("[emergency] cleared")
            if EMERGENCY_AUTO_RESUME and _was_running_before_emergency:
                running = True
                print("[emergency] auto-resume active")

    elif msg.topic == FEED_SPEED:
        try:
            s_raw = float(val)
            s_clamped = max(0, min(100, int(round(s_raw))))
            if s_clamped != speed_pct:
                prev = speed_pct
                speed_pct = s_clamped
                print(f"[speed] changed: {prev}% -> {speed_pct}%")
            else:
                print(f"[speed] received {s_clamped}% (no change)")
        except Exception as e:
            print(f"[speed] invalid value '{val}' ({e})")

# === motors ===
def _apply_motor(a,b,c,d):
    a*=FORWARD_SIGN; b*=FORWARD_SIGN; c*=FORWARD_SIGN; d*=FORWARD_SIGN
    try:
        car.set_motor_model(a,b,c,d)
    except Exception as e:
        print("[motor] error:", e)

def _maybe_publish_motor_duty(a,b,c,d):
    global _last_pub_time_motor, _last_logged_motor_left, _last_logged_motor_right
    now = time.time()
    if now - _last_pub_time_motor < 3.0:
        return
    _last_pub_time_motor = now
    try:
        to_pct = lambda v: int(round(abs(v)*100/4095))
        left  = to_pct((a+b)//2)
        right = to_pct((c+d)//2)

        # remember latest for CSV
        _last_logged_motor_left = left
        _last_logged_motor_right = right

        if _last_pub_motor["l"] is None or abs(left - _last_pub_motor["l"]) >= 5:
            enqueue_publish(FEED_MOTOR_L, left)
            _last_pub_motor["l"] = left
        if _last_pub_motor["r"] is None or abs(right - _last_pub_motor["r"]) >= 5:
            enqueue_publish(FEED_MOTOR_R, right)
            _last_pub_motor["r"] = right
    except Exception as e:
        print("[motor_pub] err:", e)

def drive_forward_pct(p):
    v = pct_to_pwm(p if (running and not emergency_on) else 0)
    _apply_motor(v,v,v,v)
    _maybe_publish_motor_duty(v,v,v,v)

# === main loop & shutdown ===
def heartbeat(now):
    global _last_hb
    if now - _last_hb >= 60.0:
        _last_hb = now
        enqueue_publish(FEED_HEARTBEAT, "online", retain=True)

def main_loop():
    global _last_applied_speed, _log_last_write, _log_cur_path, _log_header_written
    while not STOP_EVENT.is_set():
        now = time.time()

        # motion
        if not running or emergency_on:
            safe_stop()
        else:
            drive_forward_pct(speed_pct)
            if _last_applied_speed != speed_pct:
                print(f"[manual] applying speed {speed_pct}%")
                _last_applied_speed = speed_pct

        # CSV logging every 2s with daily rotation
        if now - _log_last_write >= 2.0:
            _log_last_write = now
            path = telemetry_path()
            if _log_cur_path != path:
                _log_cur_path = path
                _log_header_written = False
            if not _log_header_written:
                ensure_csv_header(path)

            ts = iso_now()
            # sensors not available
            dist = ""       # cm
            line = ""       # state
            batt = ""       # volts
            ml = "" if _last_logged_motor_left  is None else _last_logged_motor_left
            mr = "" if _last_logged_motor_right is None else _last_logged_motor_right
            row = f'{ts},{dist},{line},{batt},{int(running)},{int(emergency_on)},{speed_pct},{ml},{mr},'
            append_csv_row(path, row)

        heartbeat(now)
        STOP_EVENT.wait(0.05)

def _shutdown_sequence():
    global _shutting_down, _car_closed
    if _shutting_down:
        return
    _shutting_down = True
    try:
        STOP_EVENT.set()
        safe_stop()
        if client:
            try:
                enqueue_publish(FEED_HEARTBEAT, "offline", retain=True)
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
            except OSError as e:
                if getattr(e, "errno", None) != 9:
                    print("[shutdown] car:", e)
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
        client.will_set(FEED_HEARTBEAT, "offline", retain=True)
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message
        client.connect("io.adafruit.com", 1883, 60)
        client.loop_start()

        start_publisher_thread()
        main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_sequence()
        print("Bye.")
