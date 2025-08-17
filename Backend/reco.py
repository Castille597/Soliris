# reco.py — IBM watsonx.ai 
from __future__ import annotations
import os, json, re, time
from typing import Dict, Any, List, Optional

try:
    from ibm_watsonx_ai import Credentials
    from ibm_watsonx_ai.foundation_models import Model
except Exception:
    Credentials = None
    Model = None


def _f(v) -> Optional[float]:
    try:
        return float(v)
    except Exception:
        return None

def _strip_code_fences(s: str) -> str:
    if not s:
        return s
    s = s.strip()
    m = re.match(r"^```(?:json)?\s*(\{.*\})\s*```$", s, flags=re.S | re.I)
    return m.group(1) if m else s

def _error_response(title: str, details: str) -> Dict[str, Any]:
    return {
        "provider": "watsonx",
        "risk": "low",
        "headline": title,
        "explanation": details,
        "primary": "Please try again in a moment.",
        "actions": ["Keep your comfortable pace and regular hydration."],
        "tags": ["unavailable"],
    }

def _missing_env_response() -> Dict[str, Any]:
    missing = [k for k in ("WATSONX_API_KEY","WATSONX_URL","WATSONX_PROJECT_ID") if not os.getenv(k)]
    return _error_response(
        "Recommendations unavailable (setup needed)",
        "IBM watsonx.ai is not configured. Missing: " + ", ".join(missing) +
        ". Set these in Backend/.env and restart the backend."
    )

def _daypart(hour: int) -> str:
    if   0 <= hour < 6:  return "night"
    if   6 <= hour < 12: return "morning"
    if  12 <= hour < 18: return "afternoon"
    return "evening"

def _build_prompt(ctx: Dict[str, Any], tel: Dict[str, Any], prof: Dict[str, Any]) -> str:
    hour = int(os.getenv("LOCAL_HOUR_OVERRIDE", str(time.localtime().tm_hour)))
    data = {
        "context": {**(ctx or {}), "local_hour": hour, "daypart": _daypart(hour)},
        "telemetry": tel or {},
        "profile": prof or {},
    }

    rules = (
        "You are a health & environment coach. Use ONLY the numbers in DATA.\n"
    "Return STRICT JSON with keys exactly: "
    "risk (low|medium|high), headline, explanation, primary, actions (array, max 5), tags (array).\n"
    "\n"
    "Also use biometric numbers in DATA (biometrics.*) together with environmental context (context.*) to tailor advice.\n"
    "Typical biometric fields may include: hr_bpm, spo2_pct, skin_temp_c or core_temp_c, steps, posture, activity. Do not invent fields.\n"
    "\n"
    "Hard rules:\n"
    "- If context.uv_index is null or < 1, NEVER mention sunscreen, UV, shade, sunlight, or sun exposure.\n"
    "- If context.aqi is null or < 100, do NOT mention air pollution protection (e.g., masks, avoiding polluted areas).\n"
    "- If context.ambient_temp < 28, do NOT give ‘heat’ mitigation tips (cool down, AC/fan) unless justified by biometrics (e.g., skin/core_temp_c ≥ 37.8, hr_bpm unusually high for current activity).\n"
    "- If context.daypart == 'night', avoid sun/outdoor UV advice entirely.\n"
    "- Biometrics can elevate risk even when context seems safe (e.g., spo2_pct ≤ 92, hr_bpm > 120 at rest); if you override a rule due to biometrics, say so in the explanation.\n"
    "- Never invent values; if a metric is missing, simply omit it.\n"
    "- explanation must be 1–2 plain sentences, no brackets, no quotes.\n"
    "- primary is one specific immediate action aligned with the actual risks.\n"
    "- actions: short, concrete, non-duplicated bullets; max 5; consistent with the numbers.\n"
    "- Do not include markdown or extra keys.\n"
    )
    user = "DATA=" + json.dumps(data, ensure_ascii=False)
    return f"<system>{rules}</system>\n<user>{user}</user>"

def _make_model() -> Model | None:
    if Credentials is None or Model is None:
        return None

    api_key  = os.getenv("WATSONX_API_KEY")
    url      = os.getenv("WATSONX_URL")            
    project  = os.getenv("WATSONX_PROJECT_ID")
    model_id = os.getenv("WATSONX_MODEL_ID", "ibm/granite-3-8b-instruct")

    if not (api_key and url and project):
        return None


    print(f"[watsonx] model={model_id}", flush=True)

    creds = Credentials(url=url, api_key=api_key)
    params = {
        "decoding_method": "greedy",
        "max_new_tokens": 240,
        "temperature": 0.2,
    }

  
    try:
        return Model(model_id=model_id, params=params,
                     credentials=creds, project_id=project)
    except Exception as e:
        print(f"[watsonx] model init failed: {e}", flush=True)
        return None
        

def _sanitize(rec: Dict[str, Any], ctx: Dict[str, Any]) -> Dict[str, Any]:
    uv  = _f((ctx or {}).get("uv_index"))
    aqi = _f((ctx or {}).get("aqi"))
    amb = _f((ctx or {}).get("ambient_temp"))
    hour = int((ctx or {}).get("local_hour") or time.localtime().tm_hour)

    def bad_uv(text: str) -> bool:
        if uv is not None and uv < 1:  
            return bool(re.search(r"\b(uv|sunscreen|spf|sun(?:light| exposure)|shade)\b", text, re.I))
        if 0 <= hour < 6:  
            return bool(re.search(r"\b(uv|sunscreen|spf|sun(?:light| exposure)|shade)\b", text, re.I))
        return False

    def bad_aqi(text: str) -> bool:
        if aqi is not None and aqi < 100:
            return bool(re.search(r"\b(mask|ffp2|n95|avoid polluted|pollution|smog)\b", text, re.I))
        return False

    def bad_heat(text: str) -> bool:
        if amb is not None and amb < 28:
            return bool(re.search(r"\b(cool down|ac|fan|heat|overheat)\b", text, re.I))
        return False

    
    rec["explanation"] = re.sub(r"[\[\]]", "", str(rec.get("explanation",""))).strip()

    
    actions_in: List[str] = rec.get("actions") or []
    actions_out: List[str] = []
    for a in actions_in:
        s = str(a).strip()
        if not s: 
            continue
        if bad_uv(s) or bad_aqi(s) or bad_heat(s):
            continue
        if s not in actions_out:
            actions_out.append(s)
   
    rec["actions"] = actions_out[:5] if actions_out else ["Keep your comfortable pace and regular hydration."]

    
    primary = str(rec.get("primary","")).strip()
    if not primary or bad_uv(primary) or bad_aqi(primary) or bad_heat(primary):
        rec["primary"] = "Keep your comfortable pace and regular hydration."

    
    r = str(rec.get("risk","")).lower()
    if r not in ("low","medium","high"):
        rec["risk"] = "low"

    
    rec["provider"] = "watsonx"
    return rec


def get_recommendation(ctx: Dict[str, Any], tel: Dict[str, Any], prof: Dict[str, Any]) -> Dict[str, Any]:
    model = _make_model()
    if model is None:
        return _missing_env_response()

    try:
        prompt = _build_prompt(ctx, tel, prof)
        raw = model.generate(prompt=prompt)
        text = (raw.get("results", [{}])[0].get("generated_text") or "").strip()
        text = _strip_code_fences(text)
        rec = json.loads(text)

        required = {"risk","headline","explanation","primary","actions","tags"}
        if not isinstance(rec, dict) or not required.issubset(rec.keys()):
            return _error_response("Recommendations unavailable", "The model returned an unexpected format.")

        rec = _sanitize(rec, ctx or {})
        return rec

    except Exception as e:
        return _error_response("Recommendations temporarily unavailable", f"IBM watsonx.ai call failed: {e}")