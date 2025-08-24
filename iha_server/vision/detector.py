import os
import numpy as np
from dataclasses import dataclass
from ultralytics import YOLO
import cv2

@dataclass
class Target:
    color: str
    confidence: float
    bbox: tuple  # x1,y1,x2,y2

class PlateDetector:
    def __init__(self, model_path: str, color_classes=("red","green","black"),
                 min_conf=0.6, stable_n=5, hsv_fallback=True):
        assert os.path.exists(model_path), f"Model not found: {model_path}"
        self.model = YOLO(model_path)
        self.color_classes = [c.lower() for c in color_classes]
        self.min_conf = float(min_conf)
        self.stable_n = int(stable_n)
        self.hsv_fallback = bool(hsv_fallback)
        self._last = []

    def _hsv_estimate(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        h_mean = float(np.mean(h))
        s_mean = float(np.mean(s))
        # crude bins
        if s_mean < 40:  # desaturated → likely black
            return "black", 0.55
        if 35 <= h_mean <= 85:
            return "green", 0.6
        if h_mean < 15 or h_mean > 165:
            return "red", 0.6
        return "black", 0.5

    def infer(self, frame) -> Target | None:
        res = self.model.predict(source=frame, verbose=False)[0]
        best = None
        for b in res.boxes:
            conf = float(b.conf[0])
            cls_id = int(b.cls[0])
            if conf < self.min_conf:
                continue
            label = res.names.get(cls_id, "")
            x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
            if label.lower() in self.color_classes:
                best = Target(color=label.lower(), confidence=conf, bbox=(x1,y1,x2,y2))
                break
            if self.hsv_fallback:
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                est_color, est_conf = self._hsv_estimate(roi)
                if est_conf >= self.min_conf:
                    best = Target(color=est_color, confidence=est_conf, bbox=(x1,y1,x2,y2))
                    break
        # stability filter
        if best:
            self._last.append(best.color)
            if len(self._last) > self.stable_n:
                self._last.pop(0)
            if len(self._last) == self.stable_n and len(set(self._last)) == 1:
                return best
        return None