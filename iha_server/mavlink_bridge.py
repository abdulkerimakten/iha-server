import time
from typing import Optional
from pymavlink import mavutil

class MavBridge:
    def __init__(self, conn_str: str):
        """Connect via pymavlink only (no DroneKit).
        Examples:
          - udp:127.0.0.1:14550
          - /dev/ttyTHS1,57600
          - /dev/ttyACM0,115200
        """
        if "," in conn_str and conn_str.startswith("/dev/"):
            device, baud = conn_str.split(",", 1)
            self.master = mavutil.mavlink_connection(device, baud=int(baud))
        else:
            self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()
        self._last_gpos = None
        self._last_fix_ok = False

    # ---------- helpers ----------
    def _wait_msg(self, types, timeout=5):
        return self.master.recv_match(type=types, blocking=True, timeout=timeout)

    def _set_mode(self, mode: str, timeout=5):
        mapping = self.master.mode_mapping()
        if mapping is None or mode not in mapping:
            raise RuntimeError(f"Mode {mode} not supported by FCU")
        mode_id = mapping[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        # wait until custom_mode matches
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self._wait_msg(["HEARTBEAT"], timeout=1)
            if hb and getattr(hb, "custom_mode", None) == mode_id:
                return
        raise TimeoutError(f"Mode change to {mode} timed out")

    def _arm_disarm(self, arm: bool, timeout=10):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0, 0,0,0,0,0,0
        )
        # wait
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self._wait_msg(["HEARTBEAT"], timeout=1)
            if hb:
                armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if armed == arm:
                    return
        raise TimeoutError("Arm/Disarm timeout")

    # ---------- public API ----------
    def wait_gps_fix(self, min_sat=6, timeout=120):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self._wait_msg(["GPS_RAW_INT","GLOBAL_POSITION_INT"], timeout=1)
            if not msg:
                continue
            if msg.get_type() == "GPS_RAW_INT":
                if msg.fix_type >= 3 and getattr(msg, "satellites_visible", 0) >= min_sat:
                    self._last_fix_ok = True
                    return
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                if msg.lat != 0 and msg.lon != 0:
                    self._last_gpos = msg
                    self._last_fix_ok = True
                    return
        raise TimeoutError("GPS fix not achieved in time")

    def upload_mission(self, waypoints: list):
        # Clear existing
        self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        # Send count
        n = len(waypoints)
        self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, n)
        sent = 0
        while sent < n:
            req = self._wait_msg(["MISSION_REQUEST_INT","MISSION_REQUEST"], timeout=5)
            if not req:
                raise TimeoutError("No MISSION_REQUEST received")
            seq = int(req.seq)
            wp = waypoints[seq]
            lat = int(float(wp["lat"]) * 1e7)
            lon = int(float(wp["lon"]) * 1e7)
            alt_mm = int(float(wp["alt"]) * 1000)
            self.master.mav.mission_item_int_send(
                self.master.target_system,
                self.master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,  # current=0, autocontinue=1
                0, 0, 0, 0,  # params 1-4 unused
                lat, lon, alt_mm
            )
            sent += 1
        # Wait for ACK
        ack = self._wait_msg(["MISSION_ACK"], timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"Mission upload failed: {ack}")

    def arm_and_takeoff(self, alt):
        self._set_mode("GUIDED")
        self._arm_disarm(True)
        # TAKEOFF
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,0,0,0, 0,0, float(alt)
        )
        # Wait until relative alt reached
        t0 = time.time()
        while time.time() - t0 < 90:
            msg = self._wait_msg(["GLOBAL_POSITION_INT"], timeout=1)
            if msg:
                rel_alt_m = float(msg.relative_alt) / 1000.0
                if rel_alt_m >= float(alt) * 0.95:
                    return
        raise TimeoutError("Takeoff altitude not reached")

    def start_mission_auto(self):
        self._set_mode("AUTO")

    def set_rtl(self):
        self._set_mode("RTL")

    def current_position(self) -> tuple[float,float]:
        msg = self._wait_msg(["GLOBAL_POSITION_INT"], timeout=1)
        if msg:
            self._last_gpos = msg
        if self._last_gpos:
            return (self._last_gpos.lat / 1e7, self._last_gpos.lon / 1e7)
        # fallback if none yet
        return (0.0, 0.0)

    def send_statustext(self, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_INFO):
        """Mirror target info to GCS/telemetry without writing to disk."""
        self.master.mav.statustext_send(severity, text.encode("utf-8")[:50])

    def is_landed(self) -> bool:
        msg = self._wait_msg(["EXTENDED_SYS_STATE","GLOBAL_POSITION_INT","HEARTBEAT"], timeout=0.5)
        if msg and msg.get_type() == "EXTENDED_SYS_STATE":
            return msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        if msg and msg.get_type() == "HEARTBEAT":
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if not armed:
                return True
        if msg and msg.get_type() == "GLOBAL_POSITION_INT":
            rel_alt_m = float(msg.relative_alt) / 1000.0
            return rel_alt_m < 0.5
        return False

    def close(self):
        try:
            self.master.close()
        except Exception:
            pass