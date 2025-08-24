import json
import socket
import time
from dataclasses import dataclass

@dataclass
class IdaStatus:
    race_active: bool
    target_locked: bool

class IdaLink:
    def __init__(self, host: str, port: int, timeout=1.0):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def announce(self):
        payload = {"msg":"iha_announce","ts":time.time()}
        self.sock.sendto(json.dumps(payload).encode(), self.addr)

    def send_target(self, color: str, lat: float, lon: float, conf: float) -> bool:
        payload = {
            "msg": "engagement_target",
            "color": color.upper(),
            "lat": lat,
            "lon": lon,
            "confidence": conf,
            "timestamp": time.time()
        }
        self.sock.sendto(json.dumps(payload).encode(), self.addr)
        # optional: wait ack
        try:
            data,_ = self.sock.recvfrom(2048)
            ack = json.loads(data.decode())
            return bool(ack.get("ok", False))
        except Exception:
            return False