from tdmclient import ClientAsync, aw
from tdmclient.clientasyncnode import ClientAsyncNode
from time import perf_counter
from enum import Enum
import numpy as np
from thymath import *
from thycal import *

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

# Tests
def straight_meter():
    with Thymio(THYMIO_482_CALIBRATION) as thymio:
        thymio.forward(1000)
        while thymio.state == 'forward':
            aw(thymio.client.sleep(0.5))
            thymio.print_pose()

def create_back_and_forth_path(length: float, turns: int):
    path = []
    for _ in range(turns):
            path.append([0.0, 0.0])
            path.append([length, 0.0])
    return np.array(path)

def create_square_path(side: float, turns: int) -> np.ndarray:
    path = []
    for _ in range(turns):
            path.append([0.0, 0.0])
            path.append([side, 0.0])
            path.append([side, side])
            path.append([0.0, side])
    return np.array(path)

def create_rect_path(width: float, height: float, turns: int) -> np.ndarray:
    path = []
    for _ in range(turns):
            path.append([0.0, 0.0])
            path.append([width, 0.0])
            path.append([width, height])
            path.append([0.0, height])
    return np.array(path)

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
            path = path[i:]

        def face() -> None:
            nonlocal thymio, path, state, millimeters, radians
            if path.shape[0] <= 1:
                return
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

        log = f'x; y; theta; v; omega\n'

        # Run state machine
        while path.shape[0] > 1:

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
            log += f'{thymio.x:.3f}; {thymio.y:.3f}; {thymio.theta:.3f}; {thymio.v:.3f}; {thymio.omega:.3f}\n'

        with open('log.csv', 'w') as file:
            file.write(log)

if __name__ == '__main__':

    try:
        follow_path(0, create_back_and_forth_path(100, 1))
    except KeyboardInterrupt:
        pass
