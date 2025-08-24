A minimal, **Teknofest‑IDA compliant** UAV helper that runs **on the UAV’s Jetson**. 
It flies an uploaded waypoint mission, detects the jury plate **color** (RED/GREEN/BLACK) with YOLO, and sends a **single target message** to the USV/GCS **before the IDA race start**. After receiving an ACK, the UAV returns and lands (RTL). **No files are saved on the UAV**; logging/recording is handled on the USV side.

---
## TL;DR
```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
cp .env.example .env  # fill in connection, model path, IDA host/port
# edit config/waypoints.json to your area
python -m iha_server.main
```
- Upload mission waypoints from **GCS → Pixhawk** as usual (outside this repo).
- The service: **GUIDED arm & takeoff → AUTO mission → detect color → send UDP to IDA → on ACK: RTL**.

---
## Key Features
- **Pure `pymavlink`** (no DroneKit). Works well with **ArduPilot**; PX4 users must ensure mode mappings exist for GUIDED/AUTO/RTL (or adapt modes).
- **YOLOv8** detection with optional **HSV fallback** if your model outputs a single `plate` class only.
- **Stability filter** (N consecutive frames) to reduce false positives.
- **UDP JSON** one‑shot target message + optional **MAVLink STATUSTEXT mirror** for GCS visibility.
- **No disk writes on UAV** (Teknofest rule‑friendly; USV does the recording).

---
## Compliance & Safety Notes (Teknofest IDA)
- **No code launching from GCS during mission load**: this service is intended to start on Jetson at boot (e.g., via systemd). GCS only shows UI and sends the official mission/commands.
- **No commands after start**: after the USV race start, **no control commands** must be sent to UAV except emergency. This repo sends the target **before** the IDA start; after that, it does not accept target updates.
- **Radio/frequency discipline**: configure race frequency/channel **before** entering the race area. Keep non‑racing radios powered off per rules.

> Always validate your team’s exact rule interpretation with the latest official document.

---
## Architecture (High‑level)
```
             +------------------+              +------------------+
Camera ----> |  YOLO Detector   | --(stable)--> |  Target Message  | --UDP--> USV/GCS
             |  (+ HSV fallback)|               | (color+lat+lon)  |
             +---------^--------+              +------------------+
                       |
                       | frames
                       v
                 +-----------+     MAVLink     +----------------+
                 | Mission   | <--------------> |  Pixhawk/FCU   |
                 | Manager   |  (GUIDED/AUTO/  | (ArduPilot rec.)|
                 | (this app)|   RTL + mission)|
                 +-----------+                  +----------------+
```

---
## Requirements
**Hardware**
- NVIDIA Jetson (Orin Nano / Xavier NX etc.) with CSI/USB camera.
- Pixhawk‑class FCU running **ArduPilot** (recommended) or PX4 (with adapted modes).
- Telemetry/Wi‑Fi link to USV/GCS for UDP target message.

**Software**
- Linux on Jetson (JetPack‑based), Python ≥ 3.8.
- `pymavlink`, `ultralytics` (YOLOv8), `opencv‑python`.
- A trained YOLO model (`.pt`/`.onnx`/TensorRT `.engine`).

---
## Installation
1. Clone the repo to Jetson and open a terminal in the repo root.
2. Create venv and install deps:
   ```bash
   python3 -m venv venv && source venv/bin/activate
   pip install -r requirements.txt
   ```
3. Copy and edit env:
   ```bash
   cp .env.example .env
   # then edit .env with your serial/udp connection, model path, camera source, IDA host/port
   ```
4. Put your model under `models/` and update `YOLO_MODEL` path accordingly.
5. Fill `config/waypoints.json` with `{lat, lon, alt}` items (WGS‑84, degrees; alt in meters AGL).

---
## Configuration
### `.env` (environment variables)
| Key | Required | Example | Description |
|---|---|---|---|
| `VEHICLE_CONN` | ✓ | `udp:127.0.0.1:14550` or `/dev/ttyTHS1,57600` | MAVLink connection to FCU. For serial, include baud as shown. |
| `TAKEOFF_ALT` | ✓ | `20` | Takeoff altitude (m, relative). |
| `YOLO_MODEL` | ✓ | `models/plate_yolov8n.pt` | Path to your YOLO model. |
| `CAMERA_SOURCE` | ✓ | `/dev/video0` | OpenCV source (device path or RTSP URL). |
| `MIN_CONF` |  | `0.6` | Minimum confidence to accept a detection. |
| `STABLE_N` |  | `5` | Number of consecutive frames with same color needed. |
| `IDA_HOST` | ✓ | `192.168.1.50` | USV/GCS host to receive UDP. |
| `IDA_PORT` | ✓ | `5005` | UDP port. |
| `COLOR_CLASSES` |  | `red,green,black` | Class names in your model. If model has only `plate`, enable HSV fallback. |
| `HSV_FALLBACK` |  | `true` | Use HSV color estimation on detected plate ROI when class isn’t a color. |
| `MIRROR_STATUSTEXT` |  | `true` | Also send a MAVLink STATUSTEXT line (purely informational, no file I/O). |

### `config/waypoints.json`
Simple list of waypoints the FCU will fly in AUTO mode:
```json
[
  {"lat": 40.9876543, "lon": 29.1234567, "alt": 30},
  {"lat": 40.9877000, "lon": 29.1239000, "alt": 30},
  {"lat": 40.9874000, "lon": 29.1242000, "alt": 30}
]
```

### `config/params.yaml` (thresholds)
```yaml
vision:
  min_conf: 0.6      # same as MIN_CONF unless overridden in .env
  stable_n: 5        # same as STABLE_N
  hsv_fallback: true # same as HSV_FALLBACK
mav:
  takeoff_alt: 20    # same as TAKEOFF_ALT
  guided_timeout_s: 15
  rtl_after_ack: true
comms:
  ida_port: 5005
  heartbeat_period_s: 1.0
```

> Values in `.env` take precedence over `params.yaml`.

---
## How It Works (Runtime)
1. Connects to FCU via MAVLink (`VEHICLE_CONN`), waits for GPS fix (3D, enough sats).
2. Uploads waypoints (`MISSION_COUNT`/`MISSION_REQUEST_INT` handshake).
3. Switches to **GUIDED**, **arms**, **TAKEOFF** to `TAKEOFF_ALT`.
4. Switches to **AUTO** to fly the uploaded mission.
5. Continuously reads camera frames, runs YOLO (and optional HSV fallback), applies stability filter.
6. When a stable color is found: read current lat/lon from `GLOBAL_POSITION_INT` and **send UDP JSON** to USV/GCS.
7. If an **ACK** is received from USV: switch to **RTL** and land. If not, it will try again on the next stable detection.
8. After landing, releases resources and exits.

---
## Message Format (UAV → USV/GCS)
**UDP JSON:**
```json
{
  "msg": "engagement_target",
  "color": "RED",
  "lat": 40.1234567,
  "lon": 29.1234567,
  "confidence": 0.92,
  "timestamp": 1693212345.123
}
```
**ACK (USV → UAV):**
```json
{"ok": true}
```
> You can also observe a human‑readable line over MAVLink **STATUSTEXT** (optional) for GCS awareness.

---
## Running as a Service (systemd)
1. Copy and edit the unit file:
   ```bash
   sudo mkdir -p /etc/systemd/system
   sudo cp scripts/systemd/iha-server.service /etc/systemd/system/
   # If your repo path differs from %h/iha-server, edit WorkingDirectory/ExecStart accordingly
   ```
2. Enable and start:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable iha-server
   sudo systemctl start iha-server
   journalctl -u iha-server -f  # follow logs
   ```

---
## Testing Tips
- **Bench test** with props removed. Verify: GPS fix, mode changes, arm/disarm, takeoff command, AUTO transition.
- **YOLO sanity check**: run the model on sample frames to ensure color classes line up with `COLOR_CLASSES`. If your model has only `plate`, set `HSV_FALLBACK=true`.
- **UDP path**: run a simple UDP echo server on the USV/GCS host and confirm the JSON arrives; return `{ "ok": true }` to trigger RTL.
- **PX4 users**: if GUIDED/AUTO/RTL mapping is missing, adapt `_set_mode` to PX4 modes (e.g., OFFBOARD/…/RTL) and ensure the mission upload dialect matches.

---
## Troubleshooting / FAQ
**1) Cannot open video source**  
Check `CAMERA_SOURCE` path (`/dev/video0`). Ensure permissions and that another process isn’t locking the camera.

**2) No GPS fix / timeout**  
Move outdoors, check antenna orientation, verify GPS status on GCS, increase `wait_gps_fix` timeout if needed.

**3) Mission upload stuck (no MISSION_REQUEST)**  
Confirm FCU link is stable; check that `waypoints.json` is valid; ensure ArduPilot is in a state that accepts mission upload.

**4) Mode change not supported**  
Your FCU may not expose the requested mode name. Check `mode_mapping()` on your autopilot (ArduPilot OK; PX4 may require different names).

**5) YOLO is slow on Jetson**  
Use a lighter model (e.g., `yolov8n`), reduce input size, or deploy TensorRT (`.engine`). Make sure Jetson power mode is appropriate.

**6) No ACK from USV**  
Verify `IDA_HOST/IDA_PORT`, firewall rules, and that the USV listener responds with `{"ok": true}`.

**7) Wrong color detected**  
Tune `MIN_CONF`/`STABLE_N`, improve training data, or adjust HSV thresholds by modifying `detector.py`.

---
## Directory Layout
```
.
├─ README.md
├─ requirements.txt
├─ .env.example
├─ config/
│  ├─ waypoints.json
│  ├─ params.yaml
│  └─ comms.yaml
├─ models/               # put your YOLO model here (.pt/.onnx/.engine)
├─ iha_server/
│  ├─ main.py            # entrypoint
│  ├─ mission_manager.py # orchestration (takeoff → auto → detect → send → RTL)
│  ├─ mavlink_bridge.py  # pure pymavlink utilities
│  ├─ vision/
│  ├─ comms/
│  └─ utils/
└─ scripts/
   ├─ install.sh
   └─ systemd/iha-server.service
```

---
## Glossary
- **FCU**: Flight Control Unit (Pixhawk, etc.)
- **GUIDED/AUTO/RTL**: common ArduPilot modes for guided control, autonomous mission, and return‑to‑launch.
- **GCS**: Ground Control Station (Mission Planner, QGC, etc.)
- **USV/GCS**: The surface vessel and its control interface receiving the target message.

---
## Disclaimer
Use at your own risk. Always perform safe bench tests and follow all competition and local aviation rules. 
This repository provides example code and configuration; you are responsible for verifying correct behavior on your platform.