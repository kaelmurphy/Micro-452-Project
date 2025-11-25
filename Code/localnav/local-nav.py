import numpy as np
from thymio import Thymio
from thycal import THYMIO_482_CALIBRATION
from thymath import *
from enum import Enum
from tdmclient import aw

def follow_path(theta0: float, path: np.ndarray):

    class State(Enum):
        FOLLOW          = 0
        FACE_AWAY       = 1
        EXIT_PATH       = 2
        PROBE_OBSTACLE  = 3
        NUDGE_FORWARD   = 4
        END             = 5

    # Connect Thymio
    with Thymio(THYMIO_482_CALIBRATION) as thymio:

        # Set initial position and target
        thymio.set_pose(path[0][0], path[0][1], theta0)
        thymio.set_target(path[1][0], path[1][1])
        state: State = State.FOLLOW
        TARGET_TOLERANCE = 10

        # Until path is completed
        while state != State.END:
            match state:

                case State.FOLLOW:
                    if np.linalg.norm([path[1][0] - thymio.x, path[1][1] - thymio.y]) < TARGET_TOLERANCE:
                        if path.shape[0] > 2:
                            path = path[1:]
                            thymio.set_target(path[1][0], path[1][1])
                        else:
                            state = State.END
                    elif thymio.is_blocked():
                        thymio.probe_obstacle(path)
                        state = State.FACE_AWAY

                case State.FACE_AWAY:
                    if thymio.is_clear():
                        THYMIO_LENGTH = 110
                        x = int(thymio.x + THYMIO_LENGTH * np.cos(thymio.theta))
                        y = int(thymio.y + THYMIO_LENGTH * np.sin(thymio.theta))
                        thymio.set_target(x, y)
                        state = State.EXIT_PATH

                case State.EXIT_PATH:
                    if np.linalg.norm([x - thymio.x, y - thymio.y]) < TARGET_TOLERANCE:
                        thymio.probe_obstacle(path)
                        state = State.PROBE_OBSTACLE

                case State.PROBE_OBSTACLE:
                    if thymio.is_clear():
                        THYMIO_LENGTH = 110
                        x = int(thymio.x + THYMIO_LENGTH * np.cos(thymio.theta))
                        y = int(thymio.y + THYMIO_LENGTH * np.sin(thymio.theta))
                        segment = np.array([[thymio.x, thymio.y], [x, y]])
                        intersect, index = path_intersection_point(path, segment)
                        if index is not None:
                            thymio.set_target(intersect[0], intersect[1])
                        else:
                            thymio.set_target(x, y)
                        state = State.NUDGE_FORWARD

                case State.NUDGE_FORWARD:
                    if index is not None:
                        if np.linalg.norm([intersect[0] - thymio.x, intersect[1] - thymio.y]) < TARGET_TOLERANCE:
                            path = path[index:]
                            thymio.set_target(path[1][0], path[1][1])
                            state = State.FOLLOW
                    else:
                        if np.linalg.norm([x - thymio.x, y - thymio.y]) < TARGET_TOLERANCE:
                            thymio.probe_obstacle(path)
                            state = State.PROBE_OBSTACLE


            # TODO Update position with Kalman filter
            # thymio.set_pose(path[0][0], path[0][1], theta0)

            # Pace loop
            aw(thymio.client.sleep(0.2))

if __name__ == '__main__':

    path = np.array([
        [0, 0],
        [400, 0],
        [800, 400]
    ])

    try:
        follow_path(0, path)
    except KeyboardInterrupt:
        pass
