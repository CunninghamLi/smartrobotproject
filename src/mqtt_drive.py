# -*- coding: utf-8 -*-
# mqtt_drive.py â€” movement + simulated sensors, throttle-safe, graceful Ctrl+C, Emergency Stop,
# Local Data Logging, basic retries, modes manual/line (line includes obstacle rule),
# combined motor telemetry, camera/status publishing every 10s

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
    cand |= { _full(k) for k in both(key) }
    if key.startswith(f"{PREFIX}."):
        bare = key[len(f"{PREFIX}."):]
        cand |= { _full(k) for k in both(bare) }
    if key.startswith(f"{PREFIX}-dot-"):
        bare = key[len(f"{PREFIX}-dot-"):]
        cand |= { _full(k) for k in both(bare) }
    return list(cand)

def emergency_topic_variants(full_topic_with_prefix: str):
    return _topic_variants(full_topic_with_prefix)

# === Feeds ===
FEED_STARTSTOP = feed("startstop")
FEED_SPEED     = feed("speed")
FEED_EMERGENCY = feed("emergency")
EMERGENCY_TOPICS = emergency_topic_variants(FEED_EMERGENCY)

FEED_MODE      = feed("mode")

# simulated telemetry (for dashboard display)
FEED_DISTANCE  = feed("distance")  # smartpath-dot-sensor-dot-distance
FEED_LINE      = feed("line")      # smartpath-dot-sensor-dot-line
FEED_CAMERA    = feed("camera")    # smartpath-dot-camera-dot-status
FEED_MOTOR     = feed("motor")

# NEW operator input feeds (add to config.json):
# "line_input":     "smartpath-dot-input-dot-line"
# "distance_input": "smartpath-dot-input-dot-distance"
FEED_LINE_IN      = feed("line_input")
FEED_DISTANCE_IN  = feed("distance_input")

# === Freenove motor ===
from motor import Ordinary_Car

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

_last_applied_speed = None
_last_pub_time_motor = 0.0
_last_pub_motor = {"combined": None}

_shutting_down = False
_car_closed = False

# input state + freshness
last_line      = "CENTER"  # default last-known line if operator never sets it
last_distance  = None      # int cm
t_line         = 0.0
t_distance     = 0.0
STALE_SEC      = 6.0
DIST_STOP_CM   = 10        # <= 10 -> reverse, then stop
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

# === simulated sensors (DISPLAY ONLY) ===
def read_distance_cm():
    return max(0, int(60 + 40 * math.sin(time.time())))

def read_line_state():
    return ["LEFT", "CENTER", "RIGHT"][int(time.time()) % 3]

def read_camera_status():
    return "online"

def read_camera_fps():
    return 12 + int(3 * math.sin(time.time()/3.0))

# publish every 10s for distance, line, camera (telemetry only)
SENSOR_INTERVAL = 10.0
_last_sensor_pub_all = 0.0

def publish_sensors(now):
    global _last_sensor_pub_all
    if now - _last_sensor_pub_all < SENSOR_INTERVAL:
        return
    _last_sensor_pub_all = now
    try: enqueue_publish(FEED_DISTANCE, read_distance_cm())
    except: pass
    try: enqueue_publish(FEED_LINE, read_line_state())
    except: pass
    try:
        cam_payload = f"status={read_camera_status()},fps={read_camera_fps()}"
        enqueue_publish(FEED_CAMERA, cam_payload)
    except: pass

# === MQTT callbacks ===
def on_connect(c, u, flags, rc):
    print("Connected rc=", rc)
    for f in (FEED_STARTSTOP, FEED_SPEED, FEED_MODE, FEED_LINE_IN, FEED_DISTANCE_IN):
        c.subscribe(f); print("Subscribed:", f)
    for t in EMERGENCY_TOPICS:
        c.subscribe(t); print("Subscribed:", t)
    throttle_topic = f"{AIO_USER}/throttle"
    c.subscribe(throttle_topic); print("Subscribed:", throttle_topic)

def on_disconnect(c, u, rc):
    print("Disconnected rc=", rc)

def on_message(c, u, msg):
    global running, speed_pct, emergency_on, _was_running_before_emergency, current_mode
    global last_line, last_distance, t_line, t_distance

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

    elif msg.topic == FEED_MODE:
        v = val.lower()
        if   v.startswith("line"):  current_mode = "line"
        else:                       current_mode = "manual"
        print(f"[mode] -> {current_mode}")

    elif msg.topic == FEED_LINE_IN:
        v = val.upper()
        if v in ("LEFT","CENTER","RIGHT"):
            last_line = v
            t_line = time.time()
            print(f"[line_input] -> {last_line}")
        else:
            print(f"[line_input] invalid '{val}'")

    elif msg.topic == FEED_DISTANCE_IN:
        try:
            d = int(float(val))
            d = max(0, d)
            last_distance = d
            t_distance = time.time()
            print(f"[distance_input] -> {last_distance} cm")
        except Exception as e:
            print(f"[distance_input] invalid '{val}' ({e})")

# === motors ===
def _apply_motor(a,b,c,d):
    a*=FORWARD_SIGN; b*=FORWARD_SIGN; c*=FORWARD_SIGN; d*=FORWARD_SIGN
    try:
        car.set_motor_model(a,b,c,d)
    except Exception as e:
        print("[motor] error:", e)

def _apply_motor_with_sign(sign, a,b,c,d):
    """Explicit sign override for reverse so it is truly backward regardless of wiring."""
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
        if _last_pub_motor["combined"] is None or _last_pub_motor["combined"] != combined:
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
    # Force true reverse regardless of FORWARD_SIGN
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
        safe_stop(); return
    drive_forward_pct(speed_pct)

def drive_line():
    """Line tracking + obstacle rule (uses operator inputs; distance freshness required, line may be stale)."""
    if not running or emergency_on:
        safe_stop(); return

    now = time.time()

    # Safety gate: distance must be fresh
    distance_fresh = (last_distance is not None) and ((now - t_distance) <= STALE_SEC)
    if not distance_fresh:
        safe_stop(); return

    sp = max(0, speed_pct)

    # Obstacle rule
    if last_distance <= DIST_STOP_CM:
        safe_stop(); time.sleep(0.1)
        drive_backward_pct(sp if sp > 0 else 20)
        time.sleep(REV_MS/1000.0)
        safe_stop()
        return

    # Line may be stale; use last-known (default CENTER)
    pos = (last_line or "CENTER").upper()
    if pos == "CENTER":
        drive_forward_pct(sp)
    elif pos == "LEFT":
        turn_left_pct(sp)
    elif pos == "RIGHT":
        turn_right_pct(sp)
    else:
        drive_forward_pct(max(20, sp // 2))

# === main loop & shutdown ===
def main_loop():
    global _log_last_write, _log_cur_path, _log_header_written
    while not STOP_EVENT.is_set():
        now = time.time()

        if   current_mode == "manual": drive_manual()
        elif current_mode == "line"  : drive_line()
        else:                          safe_stop()

        if now - _log_last_write >= 2.0:
            _log_last_write = now
            path = telemetry_path()
            if _log_cur_path != path:
                _log_cur_path = path
                _log_header_written = False
            if not _log_header_written:
                ensure_csv_header(path)

            ts   = iso_now()
            dist = read_distance_cm()       # simulated telemetry for CSV
            line = read_line_state()
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
                car.close(); _car_closed = True
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
