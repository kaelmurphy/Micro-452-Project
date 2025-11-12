from tdmclient import ClientAsync
import numpy as np
import robot_constants as constants

class RemoteExecutor():

    DONE_POLLING_PERIOD = 0.1

    def __init__(self, programPath: str, **kwargs):
        self.done = False
        with open(programPath) as moveFile:
            source = moveFile.read()
            self.program = source.format(**kwargs)

    def run(self):

        with ClientAsync() as client:

            # Add events listener
            def on_event_received(node, event_name, event_data):
                if event_name == 'done':
                    self.done = True
            
            client.add_event_received_listener(on_event_received)

            # Lock client
            async def prog():
                with await client.lock() as node:

                    # Register events
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
                        await client.sleep(RemoteExecutor.DONE_POLLING_PERIOD)
            
            client.run_async_program(prog)

def move(millimeters: int) -> None:

    # Movement settings
    DELTA_T = 25                                                # Milliseconds
    SPEED   = 0.5 * constants.MAX_SPEED_MILLIMETERS_PER_SECOND  # Millimeters per seconds
    SUMMED_SPEEDS = 2                                           # Both motors speed summed up in aseba code
    MILLISECONDS_PER_SECONDS = 1000                             # Obvious only if written down
    INTEGRATION_CONSTANT = constants.MILLIMETERS_PER_SECOND_PER_LSB * (DELTA_T / MILLISECONDS_PER_SECONDS) / SUMMED_SPEEDS

    # Execute program remotely
    remoteExecutor = RemoteExecutor(
        'move.aesl',
        DT                  = DELTA_T,
        INV_INTEGR_CONST    = round(1 / INTEGRATION_CONSTANT),
        DISTANCE            = millimeters,
        LEFT_SPEED          = int(SPEED / constants.MILLIMETERS_PER_SECOND_PER_LSB),
        RIGHT_SPEED         = int(SPEED / constants.MILLIMETERS_PER_SECOND_PER_LSB)
    )
    remoteExecutor.run()

def turn(radians: float) -> None:

    # Movement settings
    DISTANCE = int(np.abs(radians * constants.WHEEL_PITCH_MILLIMETERS / 2)) # Millimeters
    DELTA_T = 25                                                            # Milliseconds
    SPEED   = 0.5 * constants.MAX_SPEED_MILLIMETERS_PER_SECOND              # Millimeters per seconds
    SUMMED_SPEEDS = 2                                                       # Both motors speed summed up in aseba code
    MILLISECONDS_PER_SECONDS = 1000                                         # Obvious only if written down
    INTEGRATION_CONSTANT = constants.MILLIMETERS_PER_SECOND_PER_LSB * (DELTA_T / MILLISECONDS_PER_SECONDS) / SUMMED_SPEEDS

    # Execute program remotely
    remoteExecutor = RemoteExecutor(
        'move.aesl',
        DT                  = DELTA_T,
        INV_INTEGR_CONST    = round(1 / INTEGRATION_CONSTANT),
        DISTANCE            = DISTANCE,
        LEFT_SPEED          = int(-np.sign(radians) * SPEED / constants.MILLIMETERS_PER_SECOND_PER_LSB),
        RIGHT_SPEED         = int(np.sign(radians) * SPEED / constants.MILLIMETERS_PER_SECOND_PER_LSB)
    )
    remoteExecutor.run()

def follow_path(path: np.ndarray):

    # Compute number of steps needed to complete path
    STEPS = path.shape[0] - 1
    step = 0
    currentDirection = 0
    wrap = lambda radians: (radians + np.pi) % (2 * np.pi) - np.pi
    while step < STEPS:

        # Compute next movement
        movementVector = path[step + 1] - path[step]
        movementAmplitude = int(np.linalg.norm(movementVector))
        movementAbsoluteDirection = np.angle(complex(movementVector[0], movementVector[1]))
        movementRelativeDirection = wrap(movementAbsoluteDirection - currentDirection)

        # Turn toward next position
        print(f'Turning {movementRelativeDirection:.3f} radians')
        turn(movementRelativeDirection)
        currentDirection = wrap(currentDirection + movementRelativeDirection)

        # Move to position
        print(f'Moving {movementAmplitude:.3f} mm')
        move(movementAmplitude)

        # Proceed to next waypoint
        step += 1

if __name__ == '__main__':

    # Path coordinates list (in meters)
    PATH = np.array([
        [0, 0],
        [100, 0],
        [100, -100],
        [200, -100],
        [300, 0]
    ])

    follow_path(PATH)
