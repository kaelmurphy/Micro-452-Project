# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Thymio class, handle connection, programs compilation and execution

# Imports
from tdmclient import ClientAsync
from tdmclient.clientasyncnode import ClientAsyncNode

# Thymio class
class Thymio():

    # Thymio settings
    WHEEL_PITCH         = 91.67329  # Millimeters
    DONE_POLLING_PERIOD = 0.1   # Seconds

    def __init__(self) -> None:
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
        
        # Run program
        self.client.run_async_program(self.execute)
