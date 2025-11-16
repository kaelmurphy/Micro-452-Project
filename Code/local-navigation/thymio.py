# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Thymio class, handle connection, programs compilation and execution

# Imports
from tdmclient import ClientAsync, aw
from tdmclient.clientasyncnode import ClientAsyncNode
import numpy as np

# Calibration class
class Calibration():
    """
    Calibration class for a Thymio robot

    Two values must be calibrated:

        - scale : Coder scale (um/lsb)

        - track : Wheel track (mm)
    """

    def __init__(self, scale: float, track: float) -> None:
        self._scale = scale
        self._track = track

    @property
    def scale(self) -> float:
        return self._scale
    
    @property
    def track(self) -> float:
        return self._track

# Thymio class
class Thymio():

    # Constants
    EVENTS_POLLING_PERIOD   = 0.1   # Seconds
    SEGMENT_TOLERANCE       = 5     # Millimeters

    # Constructor
    def __init__(self, calibration: Calibration) -> None:
        
        # Robot calibration
        self._calibration = calibration

        # Robot pose
        self._x = 0
        self._y = 0
        self._theta = 0

        # Event handler
        self._event = None
        self._data = None

        # Thymio client
        self._client = ClientAsync()
        self._node: ClientAsyncNode = None

    # Context manager
    def __enter__(self) -> 'Thymio':

        # Initialize client
        self._client.__enter__()
        self._client.add_event_received_listener(self.on_event_received)
        
        # Connect to node
        self._node = aw(self._client.lock())
        error = aw(self._node.register_events([
            ('done', 0),
            ('obstacle', 1),
            ('avoided', 4)
        ]))
        if error is not None:
            raise RuntimeError(f'Event registration error: {error}')
        aw(self._node.watch(events=True))
        return self

    def __exit__(self, type, value, traceback) -> None:
        aw(self._node.stop())
        aw(self._node.unlock())
        self._client.__exit__(type, value, traceback)

    # Pose getters and setters
    @property
    def x(self) -> int:
        return self._x
    
    @x.setter
    def x(self, value: int) -> None:
        self._x = max(-32768, min(value, 32767))

    @property
    def y(self) -> int:
        return self._y
    
    @y.setter
    def y(self, value: int) -> None:
        self._y = max(-32768, min(value, 32767))

    @property
    def theta(self) -> float:
        return self._theta

    @theta.setter
    def theta(self, value) -> int:
        self._theta = value

    # Event handler
    def on_event_received(self, node, event_name, event_data):

        # Capture only one event per program
        if self._event is None:
            self._event = event_name
            self._data = event_data
        else:
            return

    def run(self, program: str, **kwargs) -> tuple[str, list[int]]:

        # Load program
        with open(program) as file:
            source = file.read().format(**kwargs)
        
        # Compile program
        error = aw(self._node.compile(source))
        if error is not None:
            raise RuntimeError(f'Compilation error: {program} at line {error['error_line']}:{error['error_col']} {error['error_msg']}')
        
        # Run program
        self._event = None
        self._data = None
        error = aw(self._node.run())
        if error is not None:
            raise RuntimeError(f'Error {error['error_code']}')
        
        # Wait for an event
        while self._event is None:
            aw(self._client.sleep(Thymio.EVENTS_POLLING_PERIOD))
        aw(self._node.stop())
        return self._event, self._data

    # Programs
    def forward(self, millimeters: int) -> tuple[str, list[int]]:
        print(f'Moving {millimeters} millimeters')
        return self.run(
            'move.aesl',
            SCALE           = int(self._calibration.scale * 10000),
            TARGET          = millimeters,
            LEFT_DIRECTION  = '',
            RIGHT_DIRECTION = ''
        )

    def turn(self, radians: float) -> tuple[str, list[int]]:
        print(f'Turning {radians:.3f} radians')
        return self.run(
            'move.aesl',
            SCALE           = int(self._calibration.scale * 10000),
            TARGET          = int(np.abs(radians * self._calibration.track / 2)),
            LEFT_DIRECTION  = '' if np.sign(radians) < 0 else '-',
            RIGHT_DIRECTION = '-' if np.sign(radians) < 0 else ''
        )

    def move(self, position: np.ndarray) -> str:
        print(f'Moving to ({position[0]}, {position[1]})')

        # Compute motion vector and direction
        wrap = lambda radians: (radians + np.pi) % (2 * np.pi) - np.pi
        vector = np.array([position[0] - self.x, position[1] - self.y])
        direction = np.angle(complex(vector[0], vector[1]))

        # Turn
        radians = wrap(direction - self.theta)
        event, data = self.turn(radians)
        match event:

            case 'done':
                self.theta = wrap(self.theta + radians)

            case 'obstacle':
                radians = np.sign(radians) * data[0] * 2 / self._calibration.track
                self.theta = wrap(self.theta + radians)
                return 'obstacle'

        # Advance
        millimeters = int(np.linalg.norm(vector))
        event, data = self.forward(millimeters)
        match event:

            case 'done':
                self.x = position[0]
                self.y = position[1]

            case 'obstacle':
                self.x += (data[0] / millimeters) * (position[0] - self.x)
                self.y += (data[0] / millimeters) * (position[1] - self.y)
                return 'obstacle'
            
        # Target reached
        return 'done'

    def avoid(self, path: np.ndarray) -> int:
        print(f'Avoiding obstacle')

        # Angles fixed point conversion
        to_q15 = lambda radians: int(np.round(32767 * ((radians + np.pi) % (2 * np.pi) - np.pi) / np.pi))
        to_radians = lambda q15: np.pi * float(q15) / 32767

        # Compute path collision box


        # Avoid obstacle
        event, data = self.run(
            'avoid.aesl',

            # Initial position
            X_MM    = self.x,
            Y_MM    = self.y,
            THETA   = to_q15(self.theta),

            # Calibration
            SCALE   = int(np.round(10000 * self._calibration.scale)),
            TRACK   = int(np.round(10 * 2**15 / (np.pi * self._calibration.track))),

            # Collider geometry
            C_1     = 0,
            S_1     = 0,
            X_MIN_1 = 0,
            X_MAX_1 = 0,
            Y_MIN_1 = 0,
            Y_MAX_1 = 0,
            C_2     = 0,
            S_2     = 0,
            X_MIN_2 = 0,
            X_MAX_2 = 0,
            Y_MIN_2 = 0,
            Y_MAX_2 = 0
        )
        
        # Update current position
        assert event == 'avoided'
        self.x = data[0]
        self.y = data[1]
        self.theta = to_radians(data[2])
        return data[3]
