# external.py
from __future__ import annotations
import requests, os
from typing import Dict, Tuple

CITY_COORDS: Dict[str, Tuple[float, float]] = {
    "Madrid": (40.4168, -3.7038),
    "Paris":  (48.8566,  2.3522),
    "London": (51.5074, -0.1278),
    "New York": (40.7128, -74.0060),
}

def _label_us_aqi(aqi: int | float | None) -> str:
    if aqi is None: return "-"
    aqi = int(aqi)
    if aqi <= 50:   return "Good"
    if aqi <= 100:  return "Moderate"
    if aqi <= 150:  return "Unhealthy (SG)"
    if aqi <= 200:  return "Unhealthy"
    if aqi <= 300:  return "Very Unhealthy"
    return "Hazardous"

def get_context(city: str = "Madrid") -> dict:
    """Fetch current outdoor context; keys match your Flutter code."""
    city = (city or "Madrid").title()
    lat, lon = CITY_COORDS.get(city, CITY_COORDS["Madrid"])

    try:
        w = requests.get(
            "https://api.open-meteo.com/v1/forecast",
            params={
                "latitude":  lat,
                "longitude": lon,
                "current": "temperature_2m,relative_humidity_2m,pressure_msl,wind_speed_10m,uv_index,weather_code",
                "windspeed_unit": "kmh",  
                "timezone": "auto",
            },
            timeout=8,
        )
        w.raise_for_status()
        wj = w.json()
    except Exception:
        wj = {}

    try:
        aq = requests.get(
            "https://air-quality-api.open-meteo.com/v1/air-quality",
            params={
                "latitude":  lat,
                "longitude": lon,
                "current": "us_aqi,pm2_5,pm10,ozone,nitrogen_dioxide,sulphur_dioxide,carbon_monoxide",
                "timezone": "auto",
            },
            timeout=8,
        )
        aq.raise_for_status()
        aqj = aq.json()
    except Exception:
        aqj = {}

    wc = (wj.get("current") or {})
    ac = (aqj.get("current") or {})

    aqi = ac.get("us_aqi")
    return {
        "city": city,
        "lat": lat,                    
        "lon": lon,
        "ambient_temp": wc.get("temperature_2m"),   
        "humidity": wc.get("relative_humidity_2m"), 
        "pressure": wc.get("pressure_msl"),         
        "wind_speed": wc.get("wind_speed_10m"),     
        "uv_index": wc.get("uv_index"),
        "aqi": aqi,
        "air_quality": _label_us_aqi(aqi),
        "co2": None,                   
        "weather_code": wc.get("weather_code"),
    }