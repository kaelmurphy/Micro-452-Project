# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Implementation of local navigation from a given path to follow

# Imports
from thymio import Thymio, Calibration
from calibration import THYMIO_482_CALIBRATION
import numpy as np

# Navigation routine
def navigate(path: np.ndarray, calibration: Calibration) -> None:

    # Helper function to wrap angles between -PI and PI
    wrap = lambda radians: (radians + np.pi) % (2 * np.pi) - np.pi

    # Connect to thymio
    with Thymio(calibration) as thymio:

        # For each waypoint
        currentDirection = 0
        for i in range(path.shape[0] - 1):

            # Compute motion vector and direction
            waypointVector = path[i + 1] - path[i]
            waypointDirection = np.angle(complex(waypointVector[0], waypointVector[1]))

            # Turn toward waypoint
            radians = wrap(waypointDirection - currentDirection)
            thymio.turn(radians)
            currentDirection = wrap(currentDirection + radians)

            # Move to waypoint
            millimeters = int(np.linalg.norm(waypointVector))
            thymio.forward(millimeters)

            # Next waypoint
            i += 1

# Test function
def navigate_eight(calibration: Calibration) -> None:

    square = []
    for _ in range(4):
        square.append([0,   0])
        square.append([200, 0])
        square.append([200, 200])
        square.append([0,   200])
        square.append([0,   400])
        square.append([200, 400])
        square.append([200, 200])
        square.append([0,   200])
        square.append([0,   0])

    navigate(np.array(square), calibration)

# Run test
if __name__ == '__main__':

    navigate_eight(THYMIO_482_CALIBRATION)
