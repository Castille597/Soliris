Soliris Backend

Production-ready Flask backend powering the Soliris platform (IoT wearable + climate context + AI recommendations).

1) What this service does

Collects real-time telemetry from the Soliris wearable (ESP32-S3).

Uses Open‑Meteo APIs (weather + air quality) to enrich sensor data with environmental context.

Calls IBM watsonx.ai to generate personalized health & sustainability recommendations.

Provides data to the mobile app/dashboard via REST and WebSocket.

Shares daily green tips designed to be elderly-friendly and actionable.

2) Current hardware limitation

At this stage, the Soliris PCB does not have a battery installed. This means:

The device cannot run independently.

It must stay connected to a computer via USB cable to send data.

Although the SIM7600G-H 4G modem is part of the hardware, 4G functionality is not yet enabled because:

Power delivery requires a Li‑ion battery (not yet installed).

4G operation consumes more energy than USB-only mode.

Future plan

Once a battery is added:

The device will connect directly via 4G LTE to post telemetry.

GNSS (GPS) will also be fully enabled.

Soliris will work as a fully autonomous wearable, no USB needed.

3) Architecture

Current (USB-only)

[ESP32-S3 + Sensors] --USB/Serial--> ingest_serial.py --HTTP--> Flask API
                                           |                        | \
                                           |                        |  \-- Open-Meteo Weather
                                           |                        |  \-- Open-Meteo Air Quality
                                           |                        \-- IBM watsonx.ai

Flask API --WebSocket--> Frontend (Flutter/React)

Future (battery + 4G + Wifi + BLE fall out)

[ESP32-S3 + Sensors + Battery + 4G] --HTTP/4G--> Flask API
                                        |              | \
                                        |              |  \-- Open-Meteo APIs
                                        |              \-- IBM watsonx.ai

Flask API --WebSocket--> Frontend (Flutter/React)



Key sensors (typical): MAX30102 (HR/SpO₂), DS18B20 (skin temp), BME280 (temp/humidity/pressure), SGP40 (VOC index), SCD40 (CO₂), SI1151 (UV/amb. light), MPU6050 (motion/steps/posture/fall-detect), GNSS.

4) Repository layout (suggested)

Backend/
  app.py                 # Flask app + routes + WebSocket (flask_sock)
  reco.py                # Recommendation engine (watsonx.ai integration)
  external.py            # Context providers (Open‑Meteo weather + air quality)
  ingest_serial.py       # Serial → HTTP forwarder for firmware telemetry
  model/
    tips_daily.yml       # "Daily action for a greener city" tips (curated)
    prompts/
      watsonx_system.md  # System prompt + safety/grounding instructions
      watsonx_user.md    # User prompt template
  storage/
    last_state.json      # Rolling snapshot (optional persistence)
    profiles.json        # Optional user profiles (age/sex/sensitivities)
  tests/
    test_api.py          # Smoke tests for endpoints
  requirements.txt       # Python deps
  .env.example           # Env var template
  README.md              # This file

Keep filenames consistent with your project; the structure above matches how this README refers to things.

5) Quick start

5.1 Prerequisites

Python 3.9+ (recommended 3.10/3.11)

pip / venv

USB access (for Serial ingest), or firmware posting directly via HTTP

5.2 Setup

cd Backend
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# Edit .env with your keys and settings

5.3 Run the API

# Development
python app.py
# The API defaults to http://127.0.0.1:5050

5.4 Ingest from the device (via Serial → HTTP)

source .venv/bin/activate
python ingest_serial.py \
  --port /dev/cu.usbmodem1101 \
  --baud 115200 \
  --endpoint http://127.0.0.1:5050/telemetry \
  --min_interval 8

Replace --port with your actual device name.

--min_interval (seconds) throttles posts to avoid spamming.

6) Environment variables (.env)

# --- Server ---
PORT=5050
HOST=0.0.0.0
FLASK_DEBUG=1

# --- Privacy / City ---
DEFAULT_CITY=MADRID "for privacy purposes we did not use the real longitude and latitude given by the GPS in the SIM7600G-H"
PRIVACY_ANON_GPS=true

# --- Open‑Meteo ---
OPEN_METEO_LAT=40.4168
OPEN_METEO_LON=-3.7038
OPEN_METEO_UNITS=metric

# --- IBM watsonx.ai ---
WATSONX_URL=https://us-south.ml.cloud.ibm.com
WATSONX_API_KEY=***
WATSONX_PROJECT_ID=***               # Project space / deployment target
WATSONX_MODEL_ID=ibm/granite-13b-instruct   # Example; pick your deployed model
WATSONX_MAX_TOKENS=512
WATSONX_TEMPERATURE=0.2

# --- Misc ---
SAVE_LAST_STATE=true

If you manage multiple cities dynamically, you can omit OPEN_METEO_LAT/LON and let external.get_context() use device GPS and/or city name.

7) Telemetry payload (from firmware)

Endpoint: POST /telemetry

Content-Type: application/json

Example payload (align with your firmware JSON):

{
  "ts": 1723830000,
  "gps": {"lat": 40.4168, "lon": -3.7038, "city": "Madrid"},
  "env": {
    "temp_c": 31.2,
    "humidity": 44,
    "pressure_hpa": 1012.3,
    "voc_index": 34,
    "co2_ppm": 820,
    "uv_index": 6.2,
    "light_lux": 1200
  },
  "bio": {"hr_bpm": 76, "spo2_pct": 98, "skin_temp_c": 36.8},
  "motion": {
    "acc_g": {"x": -0.08, "y": 0.02, "z": 0.99},
    "posture": "standing",
    "steps": 4356,
    "activity": "walk",
    "fall": false
  },
  "privacy": {"anonymize_gps": true}
}

Minimal viable fields are accepted; unknown fields are ignored. The server stamps received_at and merges with context.

Success response:

{"ok": true}

8) REST & WebSocket API

7.1 REST

GET /health → { "ok": true, "uptime": 123.4 }

GET /state → last merged snapshot (telemetry + context + recommendation excerpt)

POST /telemetry → ingest device JSON (see above)

POST /reco/compute → force recompute recommendation immediately

GET /reco → last full recommendation payload

GET /tips/daily → returns one “Daily action for a greener city” tip (deterministic per day)

cURL examples

curl http://127.0.0.1:5050/health
curl http://127.0.0.1:5050/state
curl -X POST http://127.0.0.1:5050/reco/compute
curl http://127.0.0.1:5050/reco
curl http://127.0.0.1:5050/tips/daily

7.2 WebSocket (with flask_sock)

Path: ws://127.0.0.1:5050/ws

Messages: server pushes JSON frames whenever state updates (new telemetry/context/reco).

Example client (JS)

const ws = new WebSocket("ws://127.0.0.1:5050/ws");
ws.onmessage = (e) => {
  const state = JSON.parse(e.data);
  console.log("live state", state);
};

9) Context providers (Open‑Meteo)

external.get_context() should return weather + air quality merged into a compact JSON used by the LLM and UI.

Example implementation pseudocode

# external.py
import os, requests

def _units():
    return "metric" if os.getenv("OPEN_METEO_UNITS", "metric") == "metric" else "imperial"

def _fetch_weather(lat, lon):
    url = (
        "https://api.open-meteo.com/v1/forecast"
        f"?latitude={lat}&longitude={lon}&current=temperature_2m,apparent_temperature,relative_humidity_2m,wind_speed_10m,wind_gusts_10m,uv_index"
        "&hourly=temperature_2m,relative_humidity_2m,uv_index"
    )
    r = requests.get(url, timeout=8)
    r.raise_for_status()
    j = r.json()
    cur = j.get("current", {})
    return {
        "temperature_c": cur.get("temperature_2m"),
        "apparent_c": cur.get("apparent_temperature"),
        "humidity_pct": cur.get("relative_humidity_2m"),
        "wind_speed_ms": cur.get("wind_speed_10m"),
        "wind_gust_ms": cur.get("wind_gusts_10m"),
        "uv_index": cur.get("uv_index"),
    }

def _fetch_air(lat, lon):
    url = (
        "https://air-quality-api.open-meteo.com/v1/air-quality"
        f"?latitude={lat}&longitude={lon}&current=pm10,pm2_5,carbon_monoxide,ozone,nitrogen_dioxide,sulphur_dioxide,european_aqi,us_aqi"
    )
    r = requests.get(url, timeout=8)
    r.raise_for_status()
    cur = r.json().get("current", {})
    return {
        "pm25": cur.get("pm2_5"),
        "pm10": cur.get("pm10"),
        "ozone": cur.get("ozone"),
        "no2": cur.get("nitrogen_dioxide"),
        "so2": cur.get("sulphur_dioxide"),
        "aqi_eu": cur.get("european_aqi"),
        "aqi_us": cur.get("us_aqi"),
    }

def get_context(lat=None, lon=None, city=None):
    # fallback to .env defaults
    lat = lat or os.getenv("OPEN_METEO_LAT")
    lon = lon or os.getenv("OPEN_METEO_LON")
    w = _fetch_weather(lat, lon)
    a = _fetch_air(lat, lon)
    return {"city": city or os.getenv("DEFAULT_CITY", ""), "weather": w, "air": a}

The backend calls get_context() on new telemetry (or periodically) and merges the result for UI + LLM.

10) Recommendation engine (IBM watsonx.ai)

The recommender reads the latest user state + context and crafts a prompt that produces:

A short headline

A risk level (low/medium/high) based on heat, pollution, posture, HR/SpO₂, etc.

A set of personalized actions (clear, concrete, safe), including one "Daily action for a greener city".

Tags for UI badges.

Environment: WATSONX_URL, WATSONX_API_KEY, WATSONX_PROJECT_ID, WATSONX_MODEL_ID, WATSONX_*.

Pseudocode (Python SDK)

# reco.py
import os, time
from typing import Dict, Any

# You can use the official ibm-watsonx-ai Python SDK
# pip install ibm-watsonx-ai
from ibm_watsonx_ai.foundation_models import Model

SYSTEM_PATH = "model/prompts/watsonx_system.md"
USER_PATH   = "model/prompts/watsonx_user.md"

_model = None

def _load_model():
    global _model
    if _model is None:
        _model = Model(
            model_id=os.getenv("WATSONX_MODEL_ID"),
            params={
                "decoding_method": "greedy",
                "max_new_tokens": int(os.getenv("WATSONX_MAX_TOKENS", 512)),
                "temperature": float(os.getenv("WATSONX_TEMPERATURE", 0.2)),
            },
            credentials={
                "url": os.getenv("WATSONX_URL"),
                "apikey": os.getenv("WATSONX_API_KEY"),
            },
            project_id=os.getenv("WATSONX_PROJECT_ID"),
        )
    return _model

def _render_prompt(state: Dict[str, Any]) -> str:
    system = open(SYSTEM_PATH, "r", encoding="utf-8").read()
    user   = open(USER_PATH, "r", encoding="utf-8").read()
    # Inject live JSON state into the user prompt
    return f"""{system}\n\nUser context JSON:\n```json\n{state}\n```\n\n{user}\n"""

_last = {"ts": 0, "payload": {}}

def compute_recommendation(state: Dict[str, Any]) -> Dict[str, Any]:
    model = _load_model()
    prompt = _render_prompt(state)
    out = model.generate_text(prompt=prompt)
    # Expect JSON in the LLM output; add a robust JSON parse with fallback
    payload = _safe_parse_json(out)  # implement _safe_parse_json()
    result = {
        "provider": "watsonx",
        "ts": time.time(),
        **payload,
    }
    global _last
    _last = {"ts": result["ts"], "payload": result}
    return result

def last_recommendation() -> Dict[str, Any]:
    return _last["payload"]

Keep your prompts deterministic, safe, and testable. Return a strict JSON schema so the UI stays stable.

Sample expected LLM JSON (target schema)

{
  "risk": "low",
  "headline": "Good to go, stay hydrated",
  "explanation": "UV is moderate and AQI is acceptable; HR stable.",
  "primary": "Hydrate regularly, favor shade between 12:00–16:00.",
  "actions": [
    "Wear a hat and light, breathable clothing.",
    "Use stairs instead of the elevator today (if comfortable).",
    "Choose a bus/tram for short trips instead of a car."
  ],
  "tags": ["heat", "air", "activity"],
  "daily_city_action": {
    "title": "Refill station habit",
    "body": "Bring a reusable bottle and use public refill fountains; avoid buying single-use plastic on commutes.",
    "audience": "elderly-friendly"
  }
}

11) "Daily action for a greener city" (elderly‑friendly)

The endpoint GET /tips/daily returns one tip per day. Tips are curated in model/tips_daily.yml and can be localized.

Example tips_daily.yml

- id: 1
  title: "Refill station habit"
  audience: ["elderly-friendly", "general"]
  body: "Carry a reusable bottle; use city water refill points. Avoid single-use plastic."
- id: 2
  title: "Shaded benches map"
  audience: ["elderly-friendly"]
  body: "Plan walks along shaded benches or arcades; rest every 15 minutes during heat."
- id: 3
  title: "Off-peak errands"
  audience: ["elderly-friendly"]
  body: "Do errands before 10:30 or after 18:00 when heat and ozone are lower."

The backend uses the day-of-year to pick a deterministic tip (with optional filters based on profile).

12) Profiles & personalization

Store optional profiles in storage/profiles.json with stable user_ids (e.g., the watch UID). The recommender includes these fields in the prompt.

Example

{
  "user_001": {
    "name": "Castille",
    "age": 22,
    "sex": "F",
    "sensitivities": ["heat", "pollution"],
    "mobility": "independent"
  }
}

13) Error handling & fallbacks

If Open‑Meteo or Air‑Quality calls fail → serve last known context and tag as context_stale=true.

If watsonx.ai call fails → return a safe, generic message:

{
  "provider": "watsonx",
  "risk": "low",
  "headline": "Recommendations temporarily unavailable",
  "explanation": "AI service error",
  "primary": "Please try again in a moment.",
  "actions": ["Keep a comfortable pace and hydrate."],
  "tags": ["unavailable"]
}

All failures emit logs and still broadcast a valid frame to the UI.

14) Security & privacy

No raw PII in logs. Redact phone numbers, exact coordinates if PRIVACY_ANON_GPS=true.

Only store short-lived snapshots in storage/last_state.json if SAVE_LAST_STATE=true.

Clearly label any simulated fields used for demos.

15) Testing

pytest -q

Add smoke tests for /health, /telemetry (with a minimal payload), /state, /reco (mock watsonx), /tips/daily.

16) Roadmap / TODO



17) Troubleshooting (FAQ)

The UI shows no live data.

Check ingest_serial.py is running and posting (look for 200 OK).

Confirm POST /telemetry returns { "ok": true }.

Verify WebSocket reaches the browser (network tab → WS frames).

Open‑Meteo errors

Ensure OPEN_METEO_LAT/LON (or device GPS) are valid. Try smaller timeouts or retries.

watsonx.ai errors

Check WATSONX_URL, WATSONX_API_KEY, WATSONX_PROJECT_ID, WATSONX_MODEL_ID.

Start with a small max_new_tokens and temperature=0.2. Log the first 200 chars of the raw output for debugging.

Serial port not found

On macOS, the device may be /dev/cu.usbmodem* or /dev/cu.SLAB_USBtoUART*. Run ls /dev/cu.*.

18) Minimal code excerpts

app.py (skeleton)

from __future__ import annotations
import os, time, json, threading
from typing import Dict, Any
from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_sock import Sock
from dotenv import load_dotenv
from external import get_context
import reco

load_dotenv()
app = Flask(__name__)
CORS(app)
sock = Sock(app)

STATE: Dict[str, Any] = {"ts": 0, "context": {}, "telemetry": {}, "reco": {}}
CLIENTS = set()

def log(*a):
    print(time.strftime("[%H:%M:%S]"), *a, flush=True)

def ws_broadcast_state():
    payload = json.dumps(STATE, ensure_ascii=False)
    dead = []
    for ws in list(CLIENTS):
        try:
            ws.send(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        CLIENTS.discard(ws)

@app.get("/health")
def health():
    return jsonify({"ok": True, "uptime": time.time() - STATE.get("ts", time.time())})

@app.post("/telemetry")
def telemetry():
    data = request.get_json(force=True, silent=True) or {}
    STATE["telemetry"] = data
    # Merge context (use GPS if available)
    gps = (data.get("gps") or {})
    ctx = get_context(gps.get("lat"), gps.get("lon"), (gps.get("city") or os.getenv("DEFAULT_CITY")))
    STATE["context"] = ctx
    STATE["ts"] = time.time()

    # Compute recommendation (non-blocking is also fine)
    try:
        STATE["reco"] = reco.compute_recommendation({**data, "context": ctx})
    except Exception as e:
        log("reco error:", e)
        STATE["reco"] = {
            "provider": "watsonx",
            "risk": "low",
            "headline": "Recommendations temporarily unavailable",
            "explanation": str(e),
            "primary": "Please try again in a moment.",
            "actions": ["Keep your comfortable pace and regular hydration."],
            "tags": ["unavailable"],
        }
    ws_broadcast_state()
    return jsonify({"ok": True})

@app.get("/state")
def state():
    return jsonify(STATE)

@app.get("/reco")
def get_reco():
    return jsonify(STATE.get("reco") or {})

@app.post("/reco/compute")
def force_reco():
    STATE["reco"] = reco.compute_recommendation({**STATE.get("telemetry", {}), "context": STATE.get("context", {})})
    ws_broadcast_state()
    return jsonify({"ok": True})

@app.get("/tips/daily")
def tip():
    # simple deterministic pick by day-of-year
    import datetime as dt, random, yaml
    with open("model/tips_daily.yml", "r", encoding="utf-8") as f:
        tips = yaml.safe_load(f) or []
    doy = int(dt.datetime.now().strftime("%j"))
    pick = tips[(doy - 1) % max(1, len(tips))] if tips else {"title": "Stay hydrated", "body": "Use refill stations."}
    return jsonify(pick)

@sock.route("/ws")
def ws(ws):
    CLIENTS.add(ws)
    try:
        ws.send(json.dumps(STATE, ensure_ascii=False))
        while True:
            msg = ws.receive()
            if msg is None:
                break
    finally:
        CLIENTS.discard(ws)

if __name__ == "__main__":
    app.run(host=os.getenv("HOST", "0.0.0.0"), port=int(os.getenv("PORT", 5050)))

19) License & attribution

Weather & Air Quality: Open‑Meteo (free and attribution-friendly); follow their acceptable use policies.

IBM watsonx.ai: requires an IBM Cloud account, API key, and a deployed model in your project. Respect IBM terms of use.

20) Maintainers

Primary: Castille Latour (@castille) — Soliris project lead.
         Salomee Pernollet (@salomee) - project ingenieer

Contributions welcome via PRs with tests.