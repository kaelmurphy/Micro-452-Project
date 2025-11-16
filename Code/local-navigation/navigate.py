# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Implementation of local navigation from a given path to follow

# Imports
from thymio import Thymio, Calibration
from calibration import THYMIO_482_CALIBRATION
import numpy as np

# Navigation routine
def navigate(path: np.ndarray, direction: float, calibration: Calibration) -> None:

    # Connect to thymio
    with Thymio(calibration) as thymio:

        # Set initial pose
        thymio.x = path[0][0]
        thymio.y = path[0][1]
        thymio.theta = direction

        # For each waypoint
        for i in range(1, path.shape[0]):

            # Move toward waypoint
            match thymio.move(path[i][0], path[i][1]):

                # When waypoint is reached, 
                case 'done':
                    i += 1

                # If obstacle is reached
                case 'obstacle':
                    thymio.avoid() # TODO detect if thymio passed last waypoint when moving

            # Do high level filtering and decision making
            _ = thymio.x
            _ = thymio.y
            _ = thymio.theta

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

    navigate(np.array(square), 0, calibration)

# Run test
if __name__ == '__main__':

    navigate_eight(THYMIO_482_CALIBRATION)
