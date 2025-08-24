import cv2

class VideoSource:
    def __init__(self, source):
        self.cap = cv2.VideoCapture(source)
        assert self.cap.isOpened(), f"Cannot open video source: {source}"
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    def read(self):
        ok, frame = self.cap.read()
        return ok, frame

    def release(self):
        self.cap.release()