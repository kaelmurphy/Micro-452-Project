# Author    : Killian Baillifard
# Date      : 15.11.2025
# Brief     : Calibration class and values for each robot

# Imports
from thymio import Thymio, Calibration
import numpy as np

# Robots calibrations
THYMIO_482_CALIBRATION = Calibration(3.1254, 95.000)

# Tests functions
def forward_test(calibration: Calibration) -> None:
    
    with Thymio(calibration) as thymio:
        for _ in range(10):
            thymio.forward(100)
            thymio.backward(100)

def turn_test(calibration: Calibration) -> None:

    with Thymio(calibration) as thymio:
        for _ in range(4):
            thymio.turn(np.pi)
            thymio.turn(np.pi)

# Run test
if __name__ == '__main__':

    turn_test(THYMIO_482_CALIBRATION)
