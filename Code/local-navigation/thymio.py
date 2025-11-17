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

        - scale : Steps per mm (um/lsb)

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
    
    # Conversions
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

    # Programs
    def straight(self, millimeters: float) -> tuple[str, list[int]]:
        print(f'Moving {millimeters} millimeters')
        steps = int(np.round(millimeters * self._calibration.scale))
        hi, lo = Thymio.split_int32(steps)
        return self.run(
            'move.aesl',
            TARGET_L_HI = hi,
            TARGET_L_LO = lo,
            TARGET_R_HI = hi,
            TARGET_R_LO = lo
        )

    def turn(self, radians: float) -> tuple[str, list[int]]:
        print(f'Turning {radians:.3f} radians')
        steps = int(np.round(radians * self._calibration.track * self._calibration.scale / 2))
        l_hi, l_lo = Thymio.split_int32(-steps)
        r_hi, r_lo = Thymio.split_int32(steps)
        return self.run(
            'move.aesl',
            TARGET_L_HI = l_hi,
            TARGET_L_LO = l_lo,
            TARGET_R_HI = r_hi,
            TARGET_R_LO = r_lo
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

        # Fixed point trigonometric functions
        fp_cos = lambda vector: vector[0] / np.linalg.norm(vector)
        fp_sin = lambda vector: vector[1] / np.linalg.norm(vector)
        q15_cos = lambda vector: int(np.round(32767 * fp_cos(vector)))
        q15_sin = lambda vector: int(np.round(32767 * fp_sin(vector)))

        # Point rotation function
        rotate_x = lambda vector, point: fp_cos(vector) * point[0] + fp_sin(vector) * point[1]
        rotate_y = lambda vector, point: -fp_sin(vector) * point[0] + fp_cos(vector) * point[1]

        # Compute path vectors
        vector_1 = path[1] - path[0]
        vector_2 = path[2] - path[1] if path.shape[0] > 2 else None

        # Avoid obstacle
        event, data = self.run(
            'avoid.aesl',

            # Initial position
            X_MM    = int(self.x),
            Y_MM    = int(self.y),
            THETA   = to_q15(self.theta),

            # Calibration
            SCALE   = int(np.round(10415 * self._calibration.scale)),
            TRACK   = int(np.round(11 * 2**15 / (np.pi * self._calibration.track))),

            # Collider geometry
            STATE   = (1 if np.cross(vector_1, vector_2) >= 0 else 0) if vector_2 is not None else 0,
            C_1     = q15_cos(vector_1),
            S_1     = q15_sin(vector_1),
            X_11    = int(rotate_x(vector_1, path[0])),
            X_12    = int(rotate_x(vector_1, path[1])),
            Y_1     = int(rotate_y(vector_1, path[0])),
            C_2     = q15_cos(vector_2) if vector_2 is not None else 0,
            S_2     = q15_sin(vector_2) if vector_2 is not None else 0,
            X_21    = int(rotate_x(vector_2, path[1])) if vector_2 is not None else 0,
            X_22    = int(rotate_x(vector_2, path[2])) if vector_2 is not None else 0,
            Y_2     = int(rotate_y(vector_2, path[1])) if vector_2 is not None else 0
        )
        
        # Update current position
        assert event == 'avoided'
        self.x = data[0]
        self.y = data[1]
        self.theta = to_radians(data[2])
        return data[3]
