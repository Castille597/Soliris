# app.py
from __future__ import annotations
import os, time, json, threading
import serial
from typing import Dict, Any, Optional, List, Set
from pathlib import Path

from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_sock import Sock
from dotenv import load_dotenv

from external import get_context  

def log(msg: str) -> None:
    print(time.strftime("[%H:%M:%S] "), msg, flush=True)

def _f(v):
    """to-float safe parse, else None."""
    try:
        return float(v)
    except Exception:
        return None

load_dotenv(dotenv_path=Path(__file__).with_name(".env"), override=False)

SERIAL_PORT = os.getenv("SERIAL_PORT")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "115200"))

_ser = None
def serial_open():
    global _ser
    if _ser and _ser.is_open:
        return _ser
    if not SERIAL_PORT:
        return None
    try:
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
        time.sleep(0.2)
        return _ser
    except Exception as e:
        log(f"serial open error: {e}")
        _ser = None
        return None

def send_serial_json(obj):
    ser = serial_open()
    if not ser:
        return False
    try:
        raw = (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")
        ser.write(raw); ser.flush()
        return True
    except Exception as e:
        log(f"serial write error: {e}")
        return False

PORT = int(os.getenv("PORT", "5050"))
DEFAULT_CITY = os.getenv("DEFAULT_CITY", "Madrid")
MIN_RECO_PERIOD_SECONDS = int(os.getenv("MIN_RECO_PERIOD_SECONDS", "8"))

app = Flask(__name__)
CORS(app)
sock = Sock(app)

last_context: Optional[Dict[str, Any]] = None     
last_telemetry: Optional[Dict[str, Any]] = None   
last_profile: Optional[Dict[str, Any]] = None     
last_recommendation: Optional[Dict[str, Any]] = None
last_eco_tips: Optional[str] = None               
last_reco_ts: float = 0.0

ws_clients: Set = set()
lock = threading.Lock()

def _mk_eco_tips(ctx: Dict[str, Any], _reco_ignored: Optional[Dict[str, Any]] = None) -> str:
    """
    Daily actions for a greener city — Short eco-friendly gestures (max. 4 lines).
Backward-compatible signature: The second argument is accepted but ignored.
    """
    def _f(v):
        try:
            return float(v)
        except Exception:
            return None

    amb = _f((ctx or {}).get("ambient_temp"))   # °C
    aqi = _f((ctx or {}).get("aqi"))            # AQI
    uv  = _f((ctx or {}).get("uv_index"))
    hum = _f((ctx or {}).get("humidity") or (ctx or {}).get("humidity_pct"))
    wind= _f((ctx or {}).get("wind_speed") or (ctx or {}).get("wind_kmh"))

    tips = []

    if amb is not None and amb >= 30:
        tips += [
            "• Set AC to ≥26°C and use a fan first",
            "• Close blinds by day; ventilate at night",
        ]
    else:
        tips += [
            "• Turn off standby with a power strip",
            "• Wash at 30°C and air-dry laundry",
        ]

    if hum is not None and hum < 35:
        tips.append("• Take short showers (≤3 min) to save water")
    else:
        tips.append("• Collect cool water while warming and reuse it")


    if aqi is not None and aqi >= 100:
        tips.append("• Combine errands and prefer public transport today")
    else:
        tips.append("• Walk, cycle, or take public transport for short trips")


    reuse = [
        "• Carry a reusable bottle and tote bag",
        "• Refuse single-use cutlery and lids",
        "• Use a keep-cup for coffee",
    ]
    for opt in reuse:
        if len(tips) >= 4: break
        tips.append(opt)


    seen, out = set(), []
    for t in tips:
        if t not in seen:
            out.append(t); seen.add(t)
        if len(out) == 4:
            break

    if out:
        return "\n".join(out)
    return "• Turn off unused lights and chargers\n• Wash at 30°C\n• Air-dry laundry\n• Bring your own bottle"


def ws_broadcast(payload: Dict[str, Any]) -> None:
    raw = json.dumps(payload, ensure_ascii=False)
    dead = []
    with lock:
        for ws in list(ws_clients):
            try:
                ws.send(raw)
            except Exception:
                dead.append(ws)
        for d in dead:
            ws_clients.discard(d)

def snapshot() -> Dict[str, Any]:
    with lock:
        return {
            "ts": int(time.time() * 1000),
            "context": last_context or {},
            "telemetry": last_telemetry or {},
            "profile": last_profile or {},
            "recommendation": last_recommendation or {},
            "reco": last_recommendation or {},
            "eco_tips": last_eco_tips or "",
        }

def ws_broadcast_state() -> None:
    ws_broadcast(snapshot())

@sock.route("/ws")
def ws(ws):
    with lock:
        ws_clients.add(ws)

    try:
        ws.send(json.dumps(snapshot(), ensure_ascii=False))
        while True:
            ws.receive()  
    except Exception:
        pass
    finally:
        with lock:
            ws_clients.discard(ws)


try:
    from reco import get_recommendation as llm_reco
except Exception:
    llm_reco = None 

def compute_recommendation() -> Dict[str, Any]:
    """Appelle IBM watsonx.ai et renvoie le dict de reco (ou un dict d'erreur contrôlé)."""
    if llm_reco is None:
        return {
            "risk": "low",
            "headline": "Recommendations temporarily unavailable",
            "explanation": "AI engine not configured.",
            "primary": "Keep your comfortable pace and regular hydration.",
            "actions": ["Open settings to configure WatsonX API key."],
            "tags": ["unavailable"],
        }
    with lock:
        ctx  = last_context or {}
        tel  = last_telemetry or {}
        prof = last_profile or {}
    return llm_reco(ctx, tel, prof)


@app.get("/health")
def health():
    return {"ok": True, "ts": int(time.time() * 1000)}

@app.get("/last")
def last():
    
    global last_context, last_eco_tips
    city = request.args.get("city") or os.getenv("CITY") or DEFAULT_CITY
    if last_context is None or (last_context.get("city") != city.title()):
        try:
            last_context = get_context(city)
        except Exception as e:
            log(f"get_context error: {e}")
            last_context = {"city": city.title()}

    
    last_eco_tips = _mk_eco_tips(last_context or {}, last_recommendation or {})

    with lock:
        ctx  = last_context or {}
        tel  = last_telemetry or {}
        prof = last_profile or {}
        rec  = last_recommendation or {}
        tips = last_eco_tips or ""

    return jsonify({
        "ts": int(time.time() * 1000),
        "context": ctx,
        "telemetry": tel,
        "profile": prof,
        "recommendation": rec,
        "reco": rec,
        "eco_tips": tips,
    })

@app.post("/profile")
def save_profile():
    """Register a small profile {age, sex, ...} and rebroadcast."""
    global last_profile, last_eco_tips
    body = request.get_json(force=True, silent=True) or {}
    if not isinstance(body, dict):
        body = {"raw": body}
    with lock:
        last_profile = body
        last_eco_tips = _mk_eco_tips(last_context or {}, last_recommendation or {})
    ws_broadcast_state()
    return jsonify({"ok": True})

@app.post("/telemetry")
@app.post("/api/telemetry")
def ingest_telemetry():
    """Receives telemetry (free JSON) and recalculates the reco if periodicity ok."""
    global last_telemetry, last_recommendation, last_reco_ts, last_eco_tips
    data = request.get_json(force=True, silent=True) or {}
    if not isinstance(data, dict):
        data = {"raw": data}
    data.setdefault("ts", int(time.time() * 1000))

    with lock:
        last_telemetry = data

    now = time.time()
    if now - last_reco_ts >= MIN_RECO_PERIOD_SECONDS:
        
        try:
            last_recommendation = compute_recommendation()
            log(f"recommendation updated (risk={last_recommendation.get('risk')})")
        except Exception as e:
            log(f"reco error: {e}")
            last_recommendation = {
                "risk": "low",
                "headline": "Recommendations temporarily unavailable",
                "explanation": str(e),
                "primary": "Please try again in a moment.",
                "actions": ["Keep your comfortable pace and regular hydration."],
                "tags": ["unavailable"],
            }
        last_reco_ts = now


    last_eco_tips = _mk_eco_tips(last_context or {}, last_recommendation or {})
    ws_broadcast_state()
    return jsonify({"ok": True})

@app.post("/device/ctrl")
def device_ctrl():
    data = request.get_json(force=True, silent=True) or {}
    ok = send_serial_json(data)
    log(f"/device/ctrl {data} -> serial={'ok' if ok else 'fail'}")
    return jsonify({"ok": ok})


def refresher():
    global last_context, last_recommendation, last_reco_ts, last_eco_tips
    while True:
        try:
            city = os.getenv("CITY") or DEFAULT_CITY
            ctx = get_context(city)
            with lock:
                last_context = ctx

            if time.time() - last_reco_ts >= MIN_RECO_PERIOD_SECONDS and last_telemetry:
                try:
                    last_recommendation = compute_recommendation()
                except Exception as e:
                    log(f"reco error (refresher): {e}")
                    last_recommendation = {
                        "risk": "low",
                        "headline": "Recommendations temporarily unavailable",
                        "explanation": str(e),
                        "primary": "Please try again in a moment.",
                        "actions": ["Keep your comfortable pace and regular hydration."],
                        "tags": ["unavailable"],
                    }
                last_reco_ts = time.time()

            
            last_eco_tips = _mk_eco_tips(last_context or {}, last_recommendation or {})
            ws_broadcast_state()
            log(f"pushed context for {ctx.get('city')}: T={ctx.get('ambient_temp')}°C AQI={ctx.get('aqi')}")
        except Exception as e:
            log(f"refresher error: {e}")
        time.sleep(60)

threading.Thread(target=refresher, daemon=True).start()


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=PORT, debug=True)