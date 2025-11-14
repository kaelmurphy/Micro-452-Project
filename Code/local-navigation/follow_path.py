# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Implementation of local navigation from a given path to follow

# Imports
from thymio import Thymio
import numpy as np

# Functions
def follow_path(path: np.ndarray):

    # Helper function to wrap angles between -PI and PI
    wrap = lambda radians: (radians + np.pi) % (2 * np.pi) - np.pi

    # Connect to thymio
    with Thymio() as thymio:

        # For each waypoint
        currentDirection = 0
        for i in range(path.shape[0] - 1):

            # Compute motion vector and direction
            waypointVector = path[i + 1] - path[i]
            waypointDirection = np.angle(complex(waypointVector[0], waypointVector[1]))

            # Turn toward waypoint
            radians = wrap(waypointDirection - currentDirection)
            print(f'Turning {radians:.3f} radians')
            if radians != 0:
                thymio.run_program(
                    'move.aesl',
                    TARGET      = int(np.abs(radians * Thymio.WHEEL_PITCH / 2)),
                    LEFT_SIGN   = '' if np.sign(radians) < 0 else '-',
                    RIGHT_SIGN  = '-' if np.sign(radians) < 0 else ''
                )
                currentDirection = wrap(currentDirection + radians)

            # Move to waypoint
            millimeters = int(np.linalg.norm(waypointVector))
            print(f'Moving {millimeters:.3f} mm')
            if millimeters > 0:
                thymio.run_program(
                    'move.aesl',
                    TARGET      = millimeters,
                    LEFT_SIGN   = '',
                    RIGHT_SIGN  = ''
                )

            # Next waypoint
            i += 1

if __name__ == '__main__':

    # Declare coordinates list (in millimeters)
    SQUARE = np.array([
        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [200, 0],
        [200, -200],
        [0, -200],

        [0, 0],
        [1, 0] # Small trick at the end to make the robot face the right direction
    ])

    # Make the robot follow the given path
    follow_path(SQUARE)
