from tdmclient import ClientAsync, aw
from tdmclient.clientasyncnode import ClientAsyncNode
from enum import Enum
import numpy as np
from thymath import *
from thycal import *

class Thymio():

    def __init__(self, calibration: Calibration) -> None:
        
        # Thymio client
        self.client: ClientAsync = ClientAsync()
        self.node: ClientAsyncNode = None

        # Calibration and program
        self.cal = calibration
        with open('thymio.aesl') as file:
            self.program = file.read().format(
                SCALE = self.cal.scale,
                TRACK = self.cal.track
            )

        # Robot state
        self.t: float = 0
        self.x: int = 0
        self.y: int = 0
        self.theta: float = 0
        self.v: int = 0
        self.omega: float = 0

        # Flags
        self.blocked = False
        self.clear = False

        # Logs
        self.log: str = f't (s), x (mm), y (mm), theta (rad), v (mm/s), omega (rad/s)\n'

    def __enter__(self) -> 'Thymio':

        # Initialize client
        self.client.__enter__()
        self.client.add_event_received_listener(self.on_event_received)
        
        # Connect to node
        self.node = aw(self.client.lock())
        error = aw(self.node.register_events([
            ('pos', 7),
            ('blocked', 0),
            ('clear', 0)
        ]))
        if error is not None:
            raise RuntimeError(f'Event registration error: {error}')
        aw(self.node.watch(events=True))
        
        # Compile program
        error = aw(self.node.compile(self.program))
        if error is not None:
            raise RuntimeError(f'Compilation error: {self.program} at line {error['error_line']}:{error['error_col']} {error['error_msg']}')

        # Start program
        error = aw(self.node.run())
        if error is not None:
            raise RuntimeError(f'Error {error['error_code']}')
        return self

    def __exit__(self, type, value, traceback) -> None:
        aw(self.node.stop())
        aw(self.node.unlock())
        self.client.__exit__(type, value, traceback)
        with open('log.csv', 'w') as log:
            log.write(self.log)

    def on_event_received(self, node, event_name, event_data):

        match event_name:
            case 'pos':
                self.t = event_data[0] + event_data[1] / 1000
                self.x = event_data[2]
                self.y = event_data[3]
                self.theta = np.pi * event_data[4] / 32767
                self.v = event_data[5] * (self.cal.scale / 100000)
                self.omega = np.pi * event_data[6] / 32767
                self.log += f'{self.t}, {self.x}, {self.y}, {self.theta}, {self.v}, {self.omega}\n'
            case 'blocked':
                self.blocked = True
            case 'clear':
                self.clear = True

    def set_pose(self, x: float, y: float, theta: float) -> None:
        x = int(np.round(x))
        y = int(np.round(y))
        theta = int(np.round(32767 * theta / np.pi))
        aw(self.node.set_variables({'x_mm': [x], 'y_mm': [y], 'theta': [theta]}))

    def set_target(self, x: float, y: float) -> None:
        self.blocked = False
        x = int(np.round(x))
        y = int(np.round(y))
        aw(self.node.set_variables({'state': [0], 'r_x_mm': [x], 'r_y_mm': [y]}))

    def probe_obstacle(self, path: np.ndarray) -> None:
        self.clear = False
        direction = trajectory_direction(path)
        aw(self.node.set_variables({'state': [1], 'avoid_dir': [direction * 100]}))

    def is_blocked(self) -> bool:
        blocked = self.blocked
        self.blocked = False
        return blocked
    
    def is_clear(self) -> bool:
        clear = self.clear
        self.clear = False
        return clear

# Tests
def create_back_and_forth_path(length: int, turns: int):
    path = []
    for _ in range(turns):
            path.append([0, 0])
            path.append([length, 0])
    return np.array(path)

def create_square_path(side: int, turns: int) -> np.ndarray:
    path = []
    for _ in range(turns):
            path.append([0, 0])
            path.append([side, 0])
            path.append([side, side])
            path.append([0, side])
    return np.array(path)

def create_rect_path(width: int, height: int, turns: int) -> np.ndarray:
    path = []
    for _ in range(turns):
            path.append([0, 0])
            path.append([width, 0])
            path.append([width, height])
            path.append([0, height])
    return np.array(path)

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
