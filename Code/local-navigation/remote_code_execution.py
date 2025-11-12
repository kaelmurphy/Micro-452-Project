from tdmclient import ClientAsync

MOVE_PROGRAM_PATH = 'move.aesl'
DONE_POLLING_PERIOD = 0.1

if __name__ == "__main__":

    with ClientAsync() as client:

        # Add event listener
        done = False
        def on_event_received(node, event_name, event_data):
            global done
            if event_name == 'done':
                done = True
        
        client.add_event_received_listener(on_event_received)

        # Read aseba program
        with open(MOVE_PROGRAM_PATH) as moveFile:
            moveProgram = moveFile.read()

        # Lock client
        async def prog():
            global done
            with await client.lock() as node:

                # Register events
                error = await node.register_events([("done", 0)])
                if error is not None:
                    print(f"Event registration error: {error}")
                    return
                
                # Compile program
                error = await node.compile(moveProgram)
                if error is not None:
                    print(f"Compilation error: {error['error_msg']}")
                    return
                
                # Start program
                print('Starting program')
                await node.watch(events=True)
                error = await node.run()
                if error is not None:
                    print(f"Error {error['error_code']}")
                    return
                
                # Wait until program is done
                while not done:
                    await client.sleep(DONE_POLLING_PERIOD)
                print('Program done')
        
        client.run_async_program(prog)
