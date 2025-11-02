# -*- coding: utf-8 -*-
# mqtt_drive.py â€” minimal movement, throttle-safe, graceful Ctrl+C + Emergency Stop (multi-topic)

import os, sys, time, json, signal, threading, collections, atexit
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

# â”€â”€ .env â”€â”€
load_dotenv()
AIO_USER = os.getenv("AIO_USERNAME", "").strip()
AIO_KEY  = os.getenv("AIO_KEY", "").strip()
PREFIX   = os.getenv("AIO_PREFIX", "smartpath").strip()
if not AIO_USER or not AIO_KEY:
    print("ERROR: Missing AIO_USERNAME or AIO_KEY in .env")
    sys.exit(1)

# â”€â”€ config.json â”€â”€
CFG_PATH = os.path.join(os.path.dirname(__file__), "config.json")
with open(CFG_PATH, "r", encoding="utf-8") as f:
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

def emergency_topic_variants(full_topic_with_prefix: str) -> list[str]:
    """
    Given 'user/feeds/smartpath-dot-robot-dot-emergency', return all reasonable variants:
      - with prefix (smartpath) in -dot- and dotted forms
      - without prefix in -dot- and dotted forms
    """
    try:
        _, key = full_topic_with_prefix.split("/feeds/", 1)
    except ValueError:
        return [full_topic_with_prefix]

    # Normalize: compute both -dot- and dotted forms for the given key
    def to_variants(k: str) -> tuple[str, str]:
        if "-dot-" in k:
            return (k, k.replace("-dot-", "."))
        else:
            return (k.replace(".", "-dot-"), k)

    with_prefix_dot, with_prefix_dotted = to_variants(key)

    # Strip leading "<PREFIX>-dot-" or "<PREFIX>." if present
    no_prefix_key = key
    if key.startswith(f"{PREFIX}-dot-"):
        no_prefix_key = key[len(f"{PREFIX}-dot-"):]
    elif key.startswith(f"{PREFIX}."):
        no_prefix_key = key[len(f"{PREFIX}."):]

    no_prefix_dot, no_prefix_dotted = to_variants(no_prefix_key)

    # Build full user topics and dedupe
    topics = {
        _full(with_prefix_dot),
        _full(with_prefix_dotted),
        _full(no_prefix_dot),
        _full(no_prefix_dotted),
    }
    return list(topics)

FEED_STARTSTOP = feed("startstop")
FEED_SPEED     = feed("speed")
FEED_EMERGENCY = feed("emergency")
EMERGENCY_TOPICS = emergency_topic_variants(FEED_EMERGENCY)

FEED_MOTOR_L   = feed("motor_l")
FEED_MOTOR_R   = feed("motor_r")
FEED_HEARTBEAT = feed("heartbeat")

# â”€â”€ Freenove motor â”€â”€
from motor import Ordinary_Car

# â”€â”€ globals/state â”€â”€
STOP_EVENT      = threading.Event()     # graceful shutdown flag
running         = False
emergency_on    = False
speed_pct       = 35
FORWARD_SIGN    = -1

# emergency auto-resume flag (default OFF)
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

def pct_to_pwm(p: int) -> int:
    p = max(0, min(100, int(p)))
    return int(round(p * 4095 / 100))

# Throttle-safe publish queue
RATE_LIMIT_SECONDS = 2.1
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
            client.publish(topic, payload, retain=retain)
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

def safe_stop():
    global _car_closed
    try:
        if car and not _car_closed:
            car.set_motor_model(0,0,0,0)
            time.sleep(0.05)
    except OSError as e:
        if getattr(e, "errno", None) != 9:  # ignore EBADF from already-closed fd
            print("[safe_stop] err:", e)
    except Exception as e:
        print("[safe_stop] err:", e)

def _is_on(v: str) -> bool:
    v = (v or "").strip().lower()
    return v in {"on","1","true","start","go","enabled","enable","yes","active"}

# â”€â”€ MQTT â”€â”€
def on_connect(c, u, flags, rc):
    print("Connected rc=", rc)
    # Subscribe to start/stop + speed
    for f in (FEED_STARTSTOP, FEED_SPEED):
        c.subscribe(f); print("Subscribed:", f)
    # Subscribe to ALL plausible emergency topics (with/without prefix, dotted/-dot-)
    for t in EMERGENCY_TOPICS:
        c.subscribe(t); print("Subscribed:", t)
    # Adafruit throttle notices
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
            print("[emergency] STOP engaged")
        else:
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

# â”€â”€ motors â”€â”€
def _apply_motor(a,b,c,d):
    a*=FORWARD_SIGN; b*=FORWARD_SIGN; c*=FORWARD_SIGN; d*=FORWARD_SIGN
    try:
        car.set_motor_model(a,b,c,d)
    except Exception as e:
        print("[motor] error:", e)

def _maybe_publish_motor_duty(a,b,c,d):
    global _last_pub_time_motor
    now = time.time()
    if now - _last_pub_time_motor < 3.0:
        return
    _last_pub_time_motor = now
    try:
        to_pct = lambda v: int(round(abs(v)*100/4095))
        left  = to_pct((a+b)//2)
        right = to_pct((c+d)//2)
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

# â”€â”€ main loop & shutdown â”€â”€
def heartbeat(now):
    global _last_hb
    if now - _last_hb >= 60.0:
        _last_hb = now
        enqueue_publish(FEED_HEARTBEAT, "online", retain=True)

def main_loop():
    global _last_applied_speed
    while not STOP_EVENT.is_set():
        now = time.time()
        if not running or emergency_on:
            safe_stop()
        else:
            drive_forward_pct(speed_pct)
            if _last_applied_speed != speed_pct:
                print(f"[manual] applying speed {speed_pct}%")
                _last_applied_speed = speed_pct
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

# â”€â”€ entry â”€â”€
if __name__ == "__main__":
    try:
        car = Ordinary_Car()

        client = mqtt.Client()
        client.username_pw_set(AIO_USER, AIO_KEY)
        # LAST WILL â€” mark offline if program dies
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
