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
        i = 1
        while i < path.shape[0]:

            # Move toward waypoint
            match thymio.move(path[i]):

                case 'done':
                    i += 1

                case 'obstacle':
                    endSegment = thymio.avoid(path[(i - 1):])
                    i += endSegment - 1

            # Do high level filtering and decision making
            print(f'thymio.x: {thymio.x}')
            print(f'thymio.y: {thymio.y}')
            print(f'thymio.theta: {thymio.theta:.3f}')

# Test paths
eight = []
for _ in range(4):
    eight.append([0,   0])
    eight.append([200, 0])
    eight.append([200, 200])
    eight.append([0,   200])
    eight.append([0,   400])
    eight.append([200, 400])
    eight.append([200, 200])
    eight.append([0,   200])
    eight.append([0,   0])

triangle = []
for _ in range(4):
    triangle.append([0,   0])
    triangle.append([400, 0])
    triangle.append([200, 200])

line = [
    [0, 0],
    [600, 0]
]

# Run test
if __name__ == '__main__':

    try:
        navigate(np.array(triangle), 0, THYMIO_482_CALIBRATION)
    except KeyboardInterrupt:
        pass
