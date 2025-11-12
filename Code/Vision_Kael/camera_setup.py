import sys
import time
import threading
import cv2

def getBackend():
    # return opencv video backend
    return cv2.CAP_DSHOW if sys.platform.startswith("win") else None

class CameraStream:
    # class that handles camera capture
    def __init__(self, index, backend=None, width=1920, height=1080, fps=30):
        # store camera parameters
        self.index = index
        self.backend = getBackend() if backend is None else backend

        # open camera stream
        if self.backend is not None:
            self.cap = cv2.VideoCapture(self.index, self.backend)
        else:
            self.cap = cv2.VideoCapture(self.index)

        # check that camera opened successfully
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera at index {self.index}.")

        # apply camera settings if provided
        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps is not None:
            self.cap.set(cv2.CAP_PROP_FPS, float(fps))

        # confirm applied settings
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rf = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Camera: {w}x{h} @ ~{rf:.0f} fps")

        # setup threading and frame storage
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.frame = None

    def start(self):
        # start the camera thread if not already running
        if self.running:
            return self
        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
        return self

    def update(self):
        # capture first frame before starting loop
        ok, fr = self.cap.read()
        if ok:
            with self.lock:
                self.frame = fr

        # loop to continuously grab frames while running
        while self.running:
            ok, fr = self.cap.read()
            # wait briefly if frame not ready
            if not ok:
                time.sleep(0.005)
                continue
            # safely update shared frame variable
            with self.lock:
                self.frame = fr

    def read(self):
        # safely return a copy of the most recent frame
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        # stop camera thread and release resources
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.cap.release()