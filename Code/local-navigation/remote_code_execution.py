from tdmclient import ClientAsync
import robot_constants as constants
import robot_convert as convert

class RemoteCodeExecutor():

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
                    print('Starting program')
                    await node.watch(events=True)
                    error = await node.run()
                    if error is not None:
                        raise RuntimeError(f'Error {error['error_code']}')
                    
                    # Wait until program is done
                    while not self.done:
                        await client.sleep(RemoteCodeExecutor.DONE_POLLING_PERIOD)
                    print('Program done')
            
            client.run_async_program(prog)

def move(distance: int) -> None:

    # Movement settings
    DELTA_T = 25                                                # Milliseconds
    SPEED   = 0.5 * constants.MAX_SPEED_MILLIMETERS_PER_SECOND  # Millimeters per seconds
    SUMMED_SPEEDS = 2                                           # Both motors speed summed up in aseba code
    MILLISECONDS_PER_SECONDS = 1000                             # Obvious only if written down
    INTEGRATION_CONSTANT = constants.MILLIMETERS_PER_SECOND_PER_LSB * (DELTA_T / MILLISECONDS_PER_SECONDS) / SUMMED_SPEEDS

    # Execute program remotely
    executor = RemoteCodeExecutor(
        'move.aesl',
        DT                  = DELTA_T,
        INV_INTEGR_CONST    = round(1 / INTEGRATION_CONSTANT),
        DISTANCE            = distance,
        LEFT_SPEED          = int(SPEED / constants.MILLIMETERS_PER_SECOND_PER_LSB),
        RIGHT_SPEED         = int(SPEED / constants.MILLIMETERS_PER_SECOND_PER_LSB)
    )
    executor.run()

if __name__ == '__main__':
    move(300)
