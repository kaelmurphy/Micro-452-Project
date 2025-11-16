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
        self.calibration = calibration

        # Robot pose
        self._x = 0
        self._y = 0
        self._theta = 0

        # Program executor
        self.programPath = None
        self.programSource = None
        self.event = None
        self.segment = 0

        # Thymio client
        self.client = ClientAsync()

    # Context manager
    def __enter__(self) -> 'Thymio':
        self.client.__enter__()
        self.client.add_event_received_listener(self.on_event_received) # TODO lock and unlock node here
        return self

    def __exit__(self, type, value, traceback) -> None:
        self.client.__exit__(type, value, traceback)

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
        # return np.pi * float(self._theta) / 32767

    @theta.setter
    def theta(self, value) -> int:
        self._theta = value
        # return int(np.round(32767 * ((value + np.pi) % (2 * np.pi) - np.pi) / np.pi))

    # Event handler
    def on_event_received(self, node, event_name, event_data):

        # Capture only one event
        if self.event is None:
            self.event = event_name
        else:
            return

        # Save estimated pose
        if self.event == 'avoided':
            self._x = event_data[0]
            self._y = event_data[1]
            self._theta = event_data[2]

    # Program executor
    async def execute(self):

        node: ClientAsyncNode
        with await self.client.lock() as node:

            # Register events
            self.event = None
            error = await node.register_events([
                ('done', 0),
                ('obstacle', 0)
            ])
            if error is not None:
                raise RuntimeError(f'Event registration error: {error}')
            
            # Compile program
            error = await node.compile(self.programSource)
            if error is not None:
                raise RuntimeError(f'Compilation error: {self.programPath} at line {error['error_line']}:{error['error_col']} {error['error_msg']}')
            
            # Start program
            await node.watch(events=True)
            error = await node.run()
            if error is not None:
                raise RuntimeError(f'Error {error['error_code']}')
            
            # Wait until program is done
            while self.event is None:
                await self.client.sleep(Thymio.EVENTS_POLLING_PERIOD)

    def run_program(self, path: str, **kwargs) -> str:

        # Load program
        self.programPath = path
        with open(path) as file:
            source = file.read()
            self.programPath = path
            self.programSource = source.format(**kwargs)
        
        # Run program
        self.client.run_async_program(self.execute)
        return self.event

    # Programs
    def astolfi(self, x: int, y: int) -> str:
        print(f'Moving to ({x}, {y})')
        return self.run_program(
            'legacy/astolfi2.aesl',

            # Initial position
            X_MM = self._x,
            Y_MM = self._y,
            THETA = self._theta,

            # Calibration
            SCALE = int(np.round(10000 * self.calibration.scale)),
            TRACK = int(np.round(10 * 2**15 / (np.pi * self.calibration.track))),

            # Target
            TARGET_X_MM = x,
            TARGET_Y_MM = y
        )
    
    def forward(self, millimeters: int) -> str:
        print(f'Moving {millimeters} millimeters')
        return self.run_program(
            'move.aesl',
            SCALE           = int(self.calibration.scale * 10000),
            TARGET          = millimeters,
            LEFT_DIRECTION  = '',
            RIGHT_DIRECTION = ''
        )

    def turn(self, radians: float) -> str:
        print(f'Turning {radians:.3f} radians')
        return self.run_program(
            'move.aesl',
            SCALE           = int(self.calibration.scale * 10000),
            TARGET          = int(np.abs(radians * self.calibration.track / 2)),
            LEFT_DIRECTION  = '' if np.sign(radians) < 0 else '-',
            RIGHT_DIRECTION = '-' if np.sign(radians) < 0 else ''
        )

    def move(self, x: int, y: int) -> str:
        print(f'Moving to ({x}, {y})')

        # Compute motion vector and direction
        wrap = lambda radians: (radians + np.pi) % (2 * np.pi) - np.pi
        vector = np.array([x - self.x, y - self.y])
        direction = np.angle(complex(vector[0], vector[1]))

        # Turn
        radians = wrap(direction - self.theta)
        self.turn(radians)

        # Advance
        millimeters = int(np.linalg.norm(vector))
        result = self.forward(millimeters)

        # Update pose
        self.x = x
        self.y = y
        self.theta = wrap(self.theta + radians)
        return result

    def avoid(self) -> str:
        print(f'Avoiding obstacle')
        return self.run_program(
            'avoid.aesl',

            # Initial position
            X_MM    = self._x,
            Y_MM    = self._y,
            THETA   = self._theta,

            # Calibration
            SCALE   = int(np.round(10000 * self.calibration.scale)),
            TRACK   = int(np.round(10 * 2**15 / (np.pi * self.calibration.track))),

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
