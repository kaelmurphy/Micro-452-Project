# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Thymio class, handle connection, programs compilation and execution

# Imports
from tdmclient import ClientAsync
from tdmclient.clientasyncnode import ClientAsyncNode
import numpy as np

# Calibration class
class Calibration():
    """
    Calibration class for a Thymio robot

    Two values must be calibrated:

        - scale : Coder scale (um/lsb)

        - pitch : Wheel pitch (mm)
    """

    def __init__(self, scale: float, pitch: float) -> None:
        self._scale = scale
        self._pitch = pitch

    @property
    def scale(self) -> float:
        return self._scale
    
    @property
    def pitch(self) -> float:
        return self._pitch

# Thymio class
class Thymio():

    DONE_POLLING_PERIOD = 0.1 # Seconds

    def __init__(self, calibration: Calibration) -> None:
        self.calibration = calibration
        self.done = False
        self.programPath = None
        self.programSource = None
        self.client = ClientAsync()

    def __enter__(self) -> 'Thymio':
        self.client.__enter__()
        self.client.add_event_received_listener(self.on_event_received)
        return self

    def __exit__(self, type, value, traceback) -> None:
        self.client.__exit__(type, value, traceback)

    def on_event_received(self, node, event_name, event_data):
        if event_name == 'done':
            self.done = True

    async def execute(self):

        node: ClientAsyncNode
        with await self.client.lock() as node:

            # Register events
            self.done = False
            error = await node.register_events([
                ('done', 0)
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
            while not self.done:
                await self.client.sleep(Thymio.DONE_POLLING_PERIOD)

    def run_program(self, path: str, **kwargs) -> None:

        # Load program
        self.programPath = path
        with open(path) as file:
            source = file.read()
            self.programPath = path
            self.programSource = source.format(**kwargs)
            print(self.programSource)
        
        # Run program
        self.client.run_async_program(self.execute)

    def forward(self, millimeters: int) -> None:
        if millimeters == 0:
            return
        print(f'Going forward {millimeters} mm')
        self.run_program(
            'move2.aesl',
            SCALE           = int(self.calibration.scale * 10000),
            TARGET          = millimeters,
            LEFT_DIRECTION  = '',
            RIGHT_DIRECTION = ''
        )

    def backward(self, millimeters: int) -> None:
        if millimeters == 0:
            return
        print(f'Going backward {millimeters} mm')
        self.run_program(
            'move2.aesl',
            SCALE           = int(self.calibration.scale * 10000),
            TARGET          = millimeters,
            LEFT_DIRECTION  = '-',
            RIGHT_DIRECTION = '-'
        )

    def turn(self, radians: float) -> None:
        if radians == 0:
            return
        print(f'Turning {radians:.3f} radians')
        self.run_program(
            'move2.aesl',
            SCALE           = int(self.calibration.scale * 10000),
            TARGET          = int(np.abs(radians * self.calibration.pitch / 2)),
            LEFT_DIRECTION  = '' if np.sign(radians) < 0 else '-',
            RIGHT_DIRECTION = '-' if np.sign(radians) < 0 else ''
        )

    def astolfi(self, x: int, y: int, theta: float) -> None:
        print(f'Going to coordinate ({x}, {y}) mm with angle {theta} radians')
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        self.run_program(
            'astolfi.aesl',

            # Initial position
            X_MM = 0,
            X_UM = 0,
            Y_MM = 0,
            Y_UM = 0,
            THETA = 0,

            # Calibration
            SCALE = int(np.round(10000 * self.calibration.scale)),
            PITCH = int(np.round(10 * 2**15 / (np.pi * self.calibration.pitch))),

            # Target
            TARGET_X_MM = x,
            TARGET_Y_MM = y,
            TARGET_THETA = int(np.round(2**15 * theta / np.pi))
        )

    def avoid(self) -> None:
        print(f'Avoiding obstacle')
        self.run_program(
            'avoid.aesl',

            # Initial position
            X_MM = 0,
            X_UM = 0,
            Y_MM = 0,
            Y_UM = 0,
            THETA = 0,

            # Calibration
            SCALE = int(np.round(10000 * self.calibration.scale)),
            PITCH = int(np.round(10 * 2**15 / (np.pi * self.calibration.pitch)))
        )
