# Backend/ingest_serial.py
import argparse, json, time, requests, sys
from serial import Serial

def main():
  p = argparse.ArgumentParser()
  p.add_argument("--port", required=True, help="ex: /dev/cu.usbmodem1101, COM5, /dev/ttyACM0")
  p.add_argument("--baud", type=int, default=115200)
  p.add_argument("--endpoint", default="http://192.168.1.26:5050/recommend")  
  p.add_argument("--min_interval", type=float, default=5.0, help="seconds between model calls")
  args = p.parse_args()

  print(f"📟 Reading {args.port} @ {args.baud} → {args.endpoint}")
  last_call = 0.0

  with Serial(args.port, args.baud, timeout=1) as ser:
    while True:
      try:
        line = ser.readline()
        if not line:
          time.sleep(0.02); continue
        s = line.decode(errors="ignore").strip()
        if "{" not in s or "}" not in s:
          continue
        s = s[s.find("{"): s.rfind("}")+1]
        try:
          sample = json.loads(s)
        except Exception:
          continue

        now = time.time()
        if now - last_call < args.min_interval:
          
          continue

        last_call = now
        r = requests.post(args.endpoint, json={"sample": sample}, timeout=10)
        if r.ok:
          data = r.json()
          rec = (data.get("recommendation") or {})
          lvl = rec.get("risk_level")
          print(f"✅ risk={lvl}  reasons={rec.get('reasons')}  actions={rec.get('actions')}")
        else:
          print("⚠️ http", r.status_code, r.text[:200])

      except KeyboardInterrupt:
        print("\nbye."); sys.exit(0)
      except Exception as e:
        print("❌", e); time.sleep(0.3)

if __name__ == "__main__":
  main()