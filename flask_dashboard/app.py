# app.py
from flask import Flask, render_template, request, jsonify
from dotenv import load_dotenv
import os
import psycopg2
from psycopg2.extras import RealDictCursor

from adafruit_client import get_latest, publish_value

load_dotenv()
app = Flask(__name__)

# Adafruit feeds
DIST_FEED = "smartpath-dot-sensor-dot-distance"
LINE_FEED = "smartpath-dot-sensor-dot-line"
CAM_FEED  = "smartpath-dot-camera-dot-status"

STARTSTOP_FEED = "smartpath-dot-startstop"
SPEED_FEED     = "smartpath-dot-speed"
MODE_FEED      = "smartpath-dot-mode"

LED_FEED    = "smartpath-dot-led"
BUZZER_FEED = "smartpath-dot-buzzer"
MOTOR_FEED  = "smartpath-dot-motor"   # text commands: forward, backward, left, right, stop

# Neon connection string from .env
# Example in .env:
# DATABASE_URL=postgresql://neondb_owner:...@ep-soft-cake-ahap4muh-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require
DB_URL = os.getenv("DATABASE_URL")

def get_db_connection():
    """
    Open a new connection to Neon.
    """
    if not DB_URL:
        raise RuntimeError("DATABASE_URL is not set in .env")
    return psycopg2.connect(DB_URL, cursor_factory=RealDictCursor)


@app.route("/")
def home():
    return render_template("home.html")


@app.route("/about")
def about():
    return render_template("about.html")


@app.route("/sensors")
def sensors():
    """
    Initial load for the live sensors page.
    It shows the latest known values and then JS will poll /api/sensor-latest.
    """
    try:
        distance = get_latest(DIST_FEED)
    except Exception:
        distance = "N/A"

    try:
        line = get_latest(LINE_FEED)
    except Exception:
        line = "N/A"

    try:
        camera_status = get_latest(CAM_FEED)
    except Exception:
        camera_status = "N/A"

    return render_template(
        "sensors.html",
        distance=distance,
        line=line,
        camera_status=camera_status
    )


@app.route("/control")
def control():
    return render_template("control.html")


@app.route("/line")
def line_page():
    return render_template("line.html")


@app.route("/obstacle")
def obstacle_page():
    return render_template("obstacle.html")


@app.route("/history", methods=["GET"])
def history():
    """
    Pick a date and load historical sensor data (distance) from Neon.
    """
    selected_date = request.args.get("date")  # YYYY-MM-DD

    labels = []
    distance_data = []

    if selected_date:
        try:
            conn = get_db_connection()
            cur = conn.cursor()

            # Adjust column names if your table is different
            cur.execute(
                """
                SELECT ts, distance_cm
                FROM sensor_readings
                WHERE ts::date = %s
                ORDER BY ts
                """,
                (selected_date,)
            )
            rows = cur.fetchall()
            cur.close()
            conn.close()

            for row in rows:
                ts = row["ts"]          # datetime
                val = row["distance_cm"]
                labels.append(ts.strftime("%H:%M:%S"))
                distance_data.append(val if val is not None else None)

        except Exception as e:
            print("[history] DB error:", e)

    return render_template(
        "history.html",
        selected_date=selected_date,
        labels=labels,
        distance_data=distance_data
    )


# ---------- API endpoints ----------

@app.post("/api/startstop")
def api_startstop():
    data = request.get_json(force=True)
    state = data.get("state", "off")
    print("[api/startstop]", state)
    try:
        publish_value(STARTSTOP_FEED, state)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.post("/api/speed")
def api_speed():
    data = request.get_json(force=True)
    speed = data.get("speed", 35)
    print("[api/speed]", speed)
    try:
        publish_value(SPEED_FEED, speed)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.post("/api/mode")
def api_mode():
    data = request.get_json(force=True)
    mode = data.get("mode", "manual")
    print("[api/mode]", mode)
    try:
        publish_value(MODE_FEED, mode)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.post("/api/led")
def api_led():
    data = request.get_json(force=True)
    cmd = data.get("cmd", "off")
    print("[api/led]", cmd)
    try:
        publish_value(LED_FEED, cmd)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.post("/api/buzzer")
def api_buzzer():
    data = request.get_json(force=True)
    cmd = data.get("cmd", "off")
    print("[api/buzzer]", cmd)
    try:
        publish_value(BUZZER_FEED, cmd)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.post("/api/motor")
def api_motor():
    data = request.get_json(force=True)
    cmd = data.get("cmd", "stop")
    print("[api/motor]", cmd)
    try:
        publish_value(MOTOR_FEED, cmd)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.get("/api/sensor-latest")
def api_sensor_latest():
    """
    Small JSON API used by sensors.html to poll live values.
    """
    try:
        distance = get_latest(DIST_FEED)
    except Exception:
        distance = None

    try:
        line = get_latest(LINE_FEED)
    except Exception:
        line = None

    try:
        camera_status = get_latest(CAM_FEED)
    except Exception:
        camera_status = None

    return jsonify(
        ok=True,
        distance=distance,
        line=line,
        camera_status=camera_status
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=True)

