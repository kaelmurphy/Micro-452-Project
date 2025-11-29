from tdmclient import ClientAsync, aw
from tdmclient.clientasyncnode import ClientAsyncNode
import numpy as np

def cross2d(a: np.ndarray, b: np.ndarray):
    return a[0] * b[1] - a[1] * b[0]

def trajectory_direction(path: np.ndarray) -> int:
    if path.shape[0] >= 3:
        return -1 if cross2d(path[1] - path[0], path[2] - path[1]) < 0 else 1
    else:
        return 1

def segments_intersection_point(s1: np.ndarray, s2: np.ndarray) -> np.ndarray | None:
    r = s1[1] - s1[0]
    s = s2[1] - s2[0]
    r_cross_s = cross2d(r, s)
    if abs(r_cross_s) < 1e-9:
        return None

    q_minus_p = s2[0] - s1[0]
    t = cross2d(q_minus_p, s) / r_cross_s
    u = cross2d(q_minus_p, r) / r_cross_s

    if 0 <= t <= 1 and 0 <= u <= 1:
        return s1[0] + t * r

    return None

def path_intersection_point(path: np.ndarray, segment: np.ndarray) -> tuple[np.ndarray | None, int | None]:
    for i in range(len(path) - 1):
        path_segment = path[i:(i + 2)]
        point = segments_intersection_point(segment, path_segment)
        if point is not None:
            return point, i
    return None, None

class Calibration():

    def __init__(self, scale: int, track: int) -> None:
        self.scale: int = scale
        self.track: int = track

class Thymio():

    def __init__(self, x0: float, y0: float, theta0: float, calibration: Calibration) -> None:
        
        # Thymio client
        self.client: ClientAsync = ClientAsync()
        self.node: ClientAsyncNode = None

        # Initial pose
        self.x = int(np.round(x0))
        self.y = int(np.round(y0))
        self.theta = int(np.round(32767 * theta0 / np.pi))

        # Calibration and program
        self.cal = calibration
        with open('motorcontrol.aesl') as file:
            self.program = file.read().format(
                X0 = self.x,
                Y0 = self.y,
                THETA0 = self.theta,
                SCALE = self.cal.scale,
                TRACK = self.cal.track
            )

        # Initial commands
        self.t: float = 0
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
    
        # Pump old events
        while self.t > 1.0:
            aw(self.client.sleep(0.01))

        # Return object
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
        self.x = int(np.round(x))
        self.y = int(np.round(y))
        self.theta = int(np.round(32767 * theta / np.pi))
        aw(self.node.set_variables({'x_mm': [self.x], 'y_mm': [self.y], 'theta': [self.theta]}))

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

THYMIO_482_CALIBRATION = Calibration(31254, 1098)
