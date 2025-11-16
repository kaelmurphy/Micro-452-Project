import sys
import threading
import cv2

def getBackend():
    return cv2.CAP_DSHOW if sys.platform.startswith("win") else None

class CameraStream:
    def __init__(self, index, backend=None, width=1920, height=1080, fps=30):
        self.index = index
        self.backend = getBackend() if backend is None else backend
        
        self.cap = cv2.VideoCapture(self.index, self.backend) if self.backend else cv2.VideoCapture(self.index)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera at index {self.index}.")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.frame = None
    
    def start(self):
        if self.running:
            return self
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self
    
    def _update(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = frame
    
    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.cap.release()