# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Thymio class, handle connection, programs compilation and execution

# Imports
from tdmclient import ClientAsync
import numpy as np

# Thymio class
class Thymio():

    # Thymio settings
    MAX_SPEED                       = 200 # Millimeter
    MAX_SPEED_LSB                   = 500 # LSB
    WHEEL_PITCH                     = 100 # Millimeters
    DONE_POLLING_PERIOD             = 0.1 # Seconds
    MILLIMETERS_PER_SECOND_PER_LSB  = MAX_SPEED / MAX_SPEED_LSB

    def __init__(self) -> None:
        self.done = False
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

        with await self.client.lock() as node:

            # Register events
            self.done = False
            error = await node.register_events([
                ('done', 0)
            ])
            if error is not None:
                raise RuntimeError(f'Event registration error: {error}')
            
            # Compile program
            error = await node.compile(self.program)
            if error is not None:
                raise RuntimeError(f'Compilation error: {error['error_msg']}')
            
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
        with open(path) as file:
            source = file.read()
            self.program = source.format(**kwargs)
        
        # Run program
        self.client.run_async_program(self.execute)

    def move(self, millimeters: int) -> None:

        # Movement settings
        DELTA_T = 25                    # Milliseconds
        SPEED   = Thymio.MAX_SPEED      # Millimeters per seconds
        SUMMED_SPEEDS = 2               # Both motors speed summed up in aseba code
        MILLISECONDS_PER_SECONDS = 1000 # Obvious only if written down
        INTEGRATION_CONSTANT = Thymio.MILLIMETERS_PER_SECOND_PER_LSB * (DELTA_T / MILLISECONDS_PER_SECONDS) / SUMMED_SPEEDS

        # Execute program remotely
        self.run_program(
            'move.aesl',
            DT                  = DELTA_T,
            INV_INTEGR_CONST    = round(1 / INTEGRATION_CONSTANT),
            DISTANCE            = millimeters,
            LEFT_SPEED          = int(SPEED / Thymio.MILLIMETERS_PER_SECOND_PER_LSB),
            RIGHT_SPEED         = int(SPEED / Thymio.MILLIMETERS_PER_SECOND_PER_LSB)
        )

    def turn(self, radians: float) -> None:

        # Movement settings
        DISTANCE = int(np.abs(radians * Thymio.WHEEL_PITCH / 2)) # Millimeters
        DELTA_T = 25                    # Milliseconds
        SPEED   = Thymio.MAX_SPEED      # Millimeters per seconds
        SUMMED_SPEEDS = 2               # Both motors speed summed up in aseba code
        MILLISECONDS_PER_SECONDS = 1000 # Obvious only if written down
        INTEGRATION_CONSTANT = Thymio.MILLIMETERS_PER_SECOND_PER_LSB * (DELTA_T / MILLISECONDS_PER_SECONDS) / SUMMED_SPEEDS

        # Execute program remotely
        self.run_program(
            'move.aesl',
            DT                  = DELTA_T,
            INV_INTEGR_CONST    = round(1 / INTEGRATION_CONSTANT),
            DISTANCE            = DISTANCE,
            LEFT_SPEED          = int(-np.sign(radians) * SPEED / Thymio.MILLIMETERS_PER_SECOND_PER_LSB),
            RIGHT_SPEED         = int(np.sign(radians) * SPEED / Thymio.MILLIMETERS_PER_SECOND_PER_LSB)
        )
