import json
import time
import yaml
import os
from dataclasses import dataclass
from .mavlink_bridge import MavBridge
from .vision.detector import PlateDetector
from .vision.video_source import VideoSource
from .comms.ida_link import IdaLink
from .utils.logger import setup_logger

@dataclass
class Settings:
    takeoff_alt: float
    min_conf: float
    stable_n: int
    hsv_fallback: bool

class MissionManager:
    def __init__(self, env, logger=None):
        self.log = logger or setup_logger("mission")
        self.env = env
        self.mav = MavBridge(env["VEHICLE_CONN"])  # connect
        with open("config/params.yaml", "r") as f:
            params = yaml.safe_load(f)
        self.cfg = Settings(
            takeoff_alt=float(env.get("TAKEOFF_ALT", params["mav"]["takeoff_alt"])),
            min_conf=float(env.get("MIN_CONF", params["vision"]["min_conf"])),
            stable_n=int(env.get("STABLE_N", params["vision"]["stable_n"])),
            hsv_fallback=str(env.get("HSV_FALLBACK", params["vision"]["hsv_fallback"]))
                         .lower() in ("1","true","yes")
        )
        self.ida = IdaLink(env["IDA_HOST"], int(env["IDA_PORT"]))
        # optional mirror over STATUSTEXT (no file writes)
        self.mirror_statustext = str(env.get("MIRROR_STATUSTEXT", "true")).lower() in ("1","true","yes")
        # vision
        color_classes = [c.strip() for c in env.get("COLOR_CLASSES","red,green,black").split(",")]
        self.detector = PlateDetector(env["YOLO_MODEL"], color_classes,
                                      self.cfg.min_conf, self.cfg.stable_n, self.cfg.hsv_fallback)
        self.cam = VideoSource(env.get("CAMERA_SOURCE", 0))

    def load_waypoints(self, path="config/waypoints.json"):
        with open(path, "r") as f:
            return json.load(f)

    def run(self):
        self.log.info("Waiting GPS fix...")
        self.mav.wait_gps_fix()
        wps = self.load_waypoints()
        self.log.info(f"Uploading mission with {len(wps)} WPs...")
        self.mav.upload_mission(wps)

        self.log.info("Arming and taking off...")
        self.mav.arm_and_takeoff(self.cfg.takeoff_alt)

        self.log.info("Switching to AUTO (mission)...")
        self.mav.start_mission_auto()

        sent = False
        # main loop: detect while mission running
        try:
            while True:
                ok, frame = self.cam.read()
                if not ok:
                    time.sleep(0.02)
                    continue
                tgt = self.detector.infer(frame)
                if (not sent) and tgt:
                    lat, lon = self.mav.current_position()
                    self.log.info(f"Stable target: {tgt.color} @ conf={tgt.confidence:.2f}")
                    # optional GCS/telemetry mirror (STATUSTEXT)
                    if self.mirror_statustext:
                        self.mav.send_statustext(
                            f"TARGET {tgt.color.upper()} conf={tgt.confidence:.2f} lat={lat:.7f} lon={lon:.7f}")
                    # Primary: UDP to IDA/GCS (USV logs/files are there)
                    self.ida.announce()
                    ack = self.ida.send_target(tgt.color, lat, lon, float(tgt.confidence))
                    if ack:
                        self.log.info("IDA ACK received — initiating RTL")
                        sent = True
                        self.mav.set_rtl()
                    else:
                        self.log.warning("No ACK from IDA; will retry on next stable detect")
                # exit if landed after RTL
                if sent and self.mav.is_landed():
                    self.log.info("Landed — mission complete.")
                    break
                time.sleep(0.02)
        finally:
            self.cam.release()
            self.mav.close()