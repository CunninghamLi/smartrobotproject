import os
import requests

AIO_USER = os.getenv("AIO_USERNAME")
AIO_KEY = os.getenv("AIO_KEY")

HEADERS = {"X-AIO-Key": AIO_KEY}

def get_latest(feed_key: str):
    url = f"https://io.adafruit.com/api/v2/{AIO_USER}/feeds/{feed_key}/data/last"
    r = requests.get(url, headers=HEADERS, timeout=5)
    r.raise_for_status()
    return r.json()["value"]

def publish_value(feed_key: str, value):
    """
    Push value to a feed.
    """
    url = f"https://io.adafruit.com/api/v2/{AIO_USER}/feeds/{feed_key}/data"
    payload = {"value": value}
    r = requests.post(url, json=payload, headers=HEADERS, timeout=5)
    r.raise_for_status()
    return r.json()

