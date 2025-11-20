# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Thymio class, handle connection, programs compilation and execution

# Imports
from tdmclient import ClientAsync, aw
from tdmclient.clientasyncnode import ClientAsyncNode
from time import perf_counter
from enum import Enum
import numpy as np

# Functions
def wrap(radians: float) -> float:
    return (radians + np.pi) % (2 * np.pi) - np.pi

def split_int32(x: int) -> tuple[int, int]:
    x &= 0xFFFFFFFF
    lo = x & 0xFFFF
    if lo >= 0x8000:
        lo -= 0x10000
    hi = (x >> 16) & 0xFFFF
    if hi >= 0x8000:
        hi -= 0x10000
    return hi, lo
    
def combine_int32(hi: int, lo: int) -> int:
    hi &= 0xFFFF
    lo &= 0xFFFF
    x = (hi << 16) | lo
    if x >= 0x80000000:
        x -= 0x100000000
    return x

def cross2d(a: np.ndarray, b: np.ndarray):
    return a[0] * b[1] - a[1] * b[0]

def trajectory_direction(x: float, y: float, path: np.ndarray) -> int:
    if path.shape[0] >= 3:
        p0 = np.array([x, y])
        p1 = path[0]
        p2 = path[1]
        return -1 if cross2d(p1 - p0, p2 - p1) < 0 else 1
    else:
        return 1

def segments_intersection_distance(s1: np.ndarray, s2: np.ndarray) -> float | None:
    r = s1[1] - s1[0]
    s = s2[1] - s2[0]
    r_cross_s = cross2d(r, s)
    if abs(r_cross_s) < 1e-9:
        return None
    q_minus_p = s2[0] - s1[0]
    t = cross2d(q_minus_p, s) / r_cross_s
    u = cross2d(q_minus_p, r) / r_cross_s
    if 0 <= t <= 1 and 0 <= u <= 1:
        seg_len = np.linalg.norm(r)
        if seg_len < 1e-12:
            return None
        return t * seg_len
    return None

def path_intersection_distance(path: np.ndarray, segment: np.ndarray) -> tuple[float | None, int | None]:
    for i in range(len(path) - 1):
        pathSegment = path[i:(i + 2)]
        d = segments_intersection_distance(segment, pathSegment)
        if d is not None:
            return d, i
    return None, None

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

class Thymio():

    def __init__(self, calibration: Calibration) -> None:
        
        # Robot calibration
        self.cal: Calibration = calibration

        # Robot pose
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        # Pose and speed tracking
        self.v: float = 0.0
        self.omega: float = 0.0
        self.l: int = 0
        self.r: int = 0
        self.t: float = 0.0

        # State
        self.state: str = None

        # Thymio client
        self.client: ClientAsync = ClientAsync()
        self.node: ClientAsyncNode = None

    def __enter__(self) -> 'Thymio':

        # Initialize client
        self.client.__enter__()
        self.client.add_event_received_listener(self.on_event_received)
        
        # Connect to node
        self.node = aw(self.client.lock())
        error = aw(self.node.register_events([
            ('done', 4),
            ('obstructed', 4),
            ('cleared', 4),
            ('probed', 4),
            ('position', 4)
        ]))
        if error is not None:
            raise RuntimeError(f'Event registration error: {error}')
        aw(self.node.watch(events=True))
        return self

    def __exit__(self, type, value, traceback) -> None:
        aw(self.node.stop())
        aw(self.node.unlock())
        self.client.__exit__(type, value, traceback)

    def print_pose(self) -> None:
        print(f'Thymio pose [x = {self.x:8.3f} mm, y = {self.y:8.3f} mm, theta = {(self.theta * 180 / np.pi):8.3f} °] speed [v = {self.v:8.3f} mm/s, omega = {(self.omega * 180 / np.pi):8.3f} °/s]')

    def on_event_received(self, node, event_name, event_data):

        # Convert robots left and right position values 
        l = combine_int32(event_data[0], event_data[1])
        r = combine_int32(event_data[2], event_data[3])
        match self.state:
            case 'left turn':
                l = -l
            case 'right turn':
                r = -r
        dl = self.cal.steps_to_mm(l - self.l)
        dr = self.cal.steps_to_mm(r - self.r)
        self.l = l
        self.r = r

        # Odometry
        ds = (dl + dr) / 2
        dtheta = (dl - dr) / self.cal.track
        self.x += ds * np.cos(self.theta + (dtheta / 2))
        self.y += ds * np.sin(self.theta + (dtheta / 2))
        self.theta = wrap(self.theta + dtheta)

        # Update current speed
        match event_name:

            # Position event, still running
            case 'position':
                t = perf_counter()
                self.v = ds / (t - self.t)
                self.omega = dtheta / (t - self.t)
                self.t = t

            # Another event, stopped
            case _:
                self.v = 0.0
                self.omega = 0.0
                self.state = event_name
                aw(self.node.stop())

    def execute(self, program: str, **kwargs) -> None:

        # Load and preprocess file
        with open(program) as file:
            source = file.read().format(**kwargs)
        
        # Compile source
        error = aw(self.node.compile(source))
        if error is not None:
            raise RuntimeError(f'Compilation error: {program} at line {error['error_line']}:{error['error_col']} {error['error_msg']}')
        
        # Reset pose and speed tracking variables
        self.l = 0
        self.r = 0
        self.t = perf_counter()

        # Run program
        error = aw(self.node.run())
        if error is not None:
            raise RuntimeError(f'Error {error['error_code']}')

    def distance_and_angle_to(self, position: np.ndarray) -> tuple[float, float]:
        vector = np.array([position[0] - self.x, position[1] - self.y])
        direction = np.angle(complex(vector[0], vector[1]))
        radians = wrap(direction - self.theta)
        millimeters = np.linalg.norm(vector)
        return millimeters, radians

    def forward(self, millimeters: float) -> None:
        print(f'Moving {millimeters} millimeters')
        l_hi, l_lo = split_int32(self.cal.mm_to_steps(millimeters))
        r_hi, r_lo = split_int32(self.cal.mm_to_steps(millimeters))
        self.state = 'forward'
        self.execute(
            'move.aesl',
            DIR_R = 1,
            DIR_L = 1,
            R_L_HI = l_hi,
            R_L_LO = l_lo,
            R_R_HI = r_hi,
            R_R_LO = r_lo
        )

    def turn(self, radians: float) -> None:
        print(f'Turning {radians:.3f} radians')
        l_hi, l_lo = split_int32(self.cal.radians_to_steps(radians))
        r_hi, r_lo = split_int32(self.cal.radians_to_steps(radians))
        self.state = 'right turn' if np.sign(radians) > 0 else 'left turn'
        self.execute(
            'move.aesl',
            DIR_L = -int(np.sign(radians)),
            DIR_R = int(np.sign(radians)),
            R_L_HI = l_hi,
            R_L_LO = l_lo,
            R_R_HI = r_hi,
            R_R_LO = r_lo
        )

    def clear(self, direction: int) -> None:
        self.state = 'right turn' if direction > 0 else 'left turn'
        self.execute(
            'clear.aesl',
            DIR_L = -direction,
            DIR_R = direction
        )

    def probe(self, direction: int) -> None:
        self.state = 'right turn' if direction > 0 else 'left turn'
        self.execute(
            'probe.aesl',
            DIR_L = -direction,
            DIR_R = direction
        )

# Robots calibrations
THYMIO_482_CALIBRATION = Calibration(3.1254, 95.000)

# Tests
def straight_meter():
    with Thymio(THYMIO_482_CALIBRATION) as thymio:
        thymio.forward(1000)
        while thymio.state == 'forward':
            aw(thymio.client.sleep(0.5))
            thymio.print_pose()

def back_and_forth():
    with Thymio(THYMIO_482_CALIBRATION) as thymio:
        for _ in range(4):
            thymio.forward(400)
            while thymio.state == 'forward':
                aw(thymio.client.sleep(0.5))
                thymio.print_pose()
                
            thymio.turn(np.pi)
            while thymio.state == 'right turn':
                aw(thymio.client.sleep(0.5))
                thymio.print_pose()

def create_square_path(side: float, turns: int) -> np.ndarray:
    square = []
    for _ in range(turns):
            square.append([0.0, 0.0])
            square.append([side, 0.0])
            square.append([side, side])
            square.append([0.0, side])
    return np.array(square)

def create_rect_path(width: float, height: float, turns: int) -> np.ndarray:
    rect = []
    for _ in range(turns):
            rect.append([0.0, 0.0])
            rect.append([width, 0.0])
            rect.append([width, height])
            rect.append([0.0, height])
    return np.array(rect)

def follow_path(theta0: float, path: np.ndarray):

    class State(Enum):
        FACE        = 0
        REACH       = 1
        TURN_AWAY   = 2
        EXIT_PATH   = 3
        PROBE       = 4
        CLEAR       = 5
        NUDGE       = 6

    THYMIO_LENGTH = 110

    # Connect Thymio
    with Thymio(THYMIO_482_CALIBRATION) as thymio:

        # Initialize state machine shared variables
        state: State = None
        millimeters: float = None
        radians: float = None
        avoidanceDirection: int = None
        index: int = None
            
        # State transitions
        def next(i: int) -> None:
            nonlocal path
            if path.size > i:
                path = path[i:]
            else:
                path = np.array([])

        def face() -> None:
            nonlocal thymio, path, state, millimeters, radians
            millimeters, radians = thymio.distance_and_angle_to(path[1])
            thymio.turn(radians)
            state = State.FACE

        def reach() -> None:
            nonlocal thymio, state, millimeters
            thymio.forward(millimeters)
            state = State.REACH
        
        def turn_away() -> None:
            nonlocal thymio, path, state, avoidanceDirection
            avoidanceDirection = -trajectory_direction(thymio.x, thymio.y, path)
            thymio.clear(avoidanceDirection)
            state = State.TURN_AWAY

        def exit_path() -> None:
            nonlocal thymio, state
            thymio.forward(THYMIO_LENGTH)
            state = State.EXIT_PATH

        def probe() -> None:
            nonlocal thymio, state, avoidanceDirection
            thymio.probe(-avoidanceDirection)
            state = State.PROBE

        def clear() -> None:
            nonlocal thymio, state, avoidanceDirection
            thymio.clear(avoidanceDirection)
            state = State.CLEAR

        def nudge() -> None:
            nonlocal thymio, path, index, state
            segment = np.array([
                [
                    thymio.x,
                    thymio.y
                ],
                [
                    thymio.x + THYMIO_LENGTH * np.cos(thymio.theta),
                    thymio.y + THYMIO_LENGTH * np.sin(thymio.theta)
                ]
            ])
            distance, index = path_intersection_distance(path, segment)
            thymio.forward(distance if distance is not None else THYMIO_LENGTH)
            state = State.NUDGE

        # Set initial coordinates and angle
        thymio.x = path[0][0]
        thymio.y = path[0][1]
        thymio.theta = theta0
        face()

        # Run state machine
        while path.size > 0:

            # Run Thymio local navigation
            match state:

                case State.FACE:
                    match thymio.state:
                        case 'done':
                            reach()
                        case 'obstructed':
                            turn_away()
                    
                case State.REACH:
                    match thymio.state:
                        case 'done':
                            next(1)
                            face()
                        case 'obstructed':
                            turn_away()
                    
                case State.TURN_AWAY:
                    match thymio.state:
                        case 'cleared':
                            exit_path()

                case State.EXIT_PATH:
                    match thymio.state:
                        case 'done':
                            probe()
                        case 'obstructed':
                            clear()

                case State.PROBE:
                    match thymio.state:
                        case 'probed':
                            clear()

                case State.CLEAR:
                    match thymio.state:
                        case 'cleared':
                            nudge()

                case State.NUDGE:
                    match thymio.state:
                        case 'done':
                            if index is not None:
                                next(index)
                                face()
                            else:
                                probe()
                        case 'obstructed':
                            clear()

            # TODO Call Kalman filter and do decision making
            aw(thymio.client.sleep(0.5))
            # thymio.print_pose()

if __name__ == '__main__':

    try:
        follow_path(0, create_rect_path(200, 600, 4))
    except KeyboardInterrupt:
        pass
