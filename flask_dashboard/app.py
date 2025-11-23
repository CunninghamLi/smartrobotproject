from flask import Flask, render_template, request, jsonify
from dotenv import load_dotenv
import os

from adafruit_client import get_latest, publish_value

load_dotenv()
app = Flask(__name__)

DIST_FEED = "smartpath-dot-sensor-dot-distance"
LINE_FEED = "smartpath-dot-sensor-dot-line"
CAM_FEED  = "smartpath-dot-camera-dot-status"

STARTSTOP_FEED = "smartpath-dot-startstop"
SPEED_FEED     = "smartpath-dot-speed"
MODE_FEED      = "smartpath-dot-mode"

# If you already have a motor direction feed, put it here
# Example name only, change to your real one if needed
MOTOR_FEED = os.getenv("MOTOR_FEED_KEY", "").strip()  # optional

@app.route("/")
def home():
    return render_template("home.html")

@app.route("/about")
def about():
    return render_template("about.html")

@app.route("/sensors")
def sensors():
    try:
        distance = get_latest(DIST_FEED)
    except:
        distance = "N/A"

    try:
        line = get_latest(LINE_FEED)
    except:
        line = "N/A"

    try:
        camera_status = get_latest(CAM_FEED)
    except:
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


# ---------- API endpoints used by pages ----------

@app.post("/api/startstop")
def api_startstop():
    data = request.get_json(force=True)
    state = data.get("state", "off")
    try:
        publish_value(STARTSTOP_FEED, state)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.post("/api/speed")
def api_speed():
    data = request.get_json(force=True)
    speed = data.get("speed", 35)
    try:
        publish_value(SPEED_FEED, speed)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.post("/api/mode")
def api_mode():
    data = request.get_json(force=True)
    mode = data.get("mode", "manual")
    try:
        publish_value(MODE_FEED, mode)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.post("/api/motor")
def api_motor():
    """
    Optional. Only works if you have a motor feed and mqtt_drive subscribes to it.
    """
    if not MOTOR_FEED:
        return jsonify(ok=False, error="MOTOR_FEED_KEY not set"), 400
    data = request.get_json(force=True)
    cmd = data.get("cmd", "stop")
    try:
        publish_value(MOTOR_FEED, cmd)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=True)

