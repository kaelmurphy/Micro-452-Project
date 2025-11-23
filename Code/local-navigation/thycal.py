class Calibration():

    def __init__(self, scale: int, track: int) -> None:
        self.scale: int = scale
        self.track: int = track
    
THYMIO_482_CALIBRATION = Calibration(31254, 1098)
