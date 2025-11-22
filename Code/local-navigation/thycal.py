import numpy as np

class Calibration():

    def __init__(self, scale: float, track: float) -> None:
        self.scale = scale # (um/lsb)
        self.track = track # (mm)

    def mm_to_steps(self, millimeters: float) -> int:
        return int(np.round(1000 * np.abs(millimeters) / self.scale))

    def steps_to_mm(self, steps: int) -> float:
        return float(steps * self.scale / 1000)
    
    def radians_to_steps(self, radians: float) -> int:
        return int(self.mm_to_steps(radians * self.track / 2))
    
    def steps_to_radians(self, steps: int) -> float:
        return float(self.steps_to_mm(steps) * 2 / self.track)
    
THYMIO_482_CALIBRATION = Calibration(3.1254, 95.000)
