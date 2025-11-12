# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Implementation of local navigation from a given path to follow

# Imports
from Thymio import Thymio
import numpy as np
import robot_convert as convert
import robot_constants as constants
from remote_code_execution import move, turn
import time

# Connection settings
PORT    = 'COM3'    # use tty on unix
DT      = 0.1       # seconds

# Control routines
def wait_connection(thymio: Thymio) -> None:

    # Wait until last key is created
    TEST_KEY = 'sd.present'
    DUMMY_DELAY = 5
    while True:
        try:
            _ = thymio[TEST_KEY]
            break
        except KeyError:
            pass
        finally:
            time.sleep(DT)

    # Wait until last key is readable
    while thymio[TEST_KEY] == []:
        time.sleep(DT)

    # Force start of communications with a dummy command
    time.sleep(DUMMY_DELAY)

def stop(thymio: Thymio) -> None:

    # Give stop command
    thymio['motor.left.target'] = 0
    thymio['motor.right.target'] = 0

    # Wait until speed fall under threshold
    SPEED_TOLERANCE = 0.001
    while True:
        time.sleep(DT)
        MOT_L_SPEED = convert.to_meters_per_seconds(thymio['motor.left.speed'])
        MOT_R_SPEED = convert.to_meters_per_seconds(thymio['motor.right.speed'])
        if MOT_L_SPEED < SPEED_TOLERANCE and MOT_R_SPEED < SPEED_TOLERANCE:
            return

def move_distance(thymio: Thymio, distance: float) -> None:

    # Control loop
    ERROR_TOLERANCE = 0.01
    position = 0
    while True:

        # Read and convert sensors values
        PROX_H      = np.array([convert.to_normalized_proximity(value) for value in thymio['prox.horizontal']])
        ACC         = np.array([convert.to_meters_per_seconds_squared(axis) for axis in thymio['acc']])
        MOT_L_SPEED = convert.to_meters_per_seconds(thymio['motor.left.speed'])
        MOT_R_SPEED = convert.to_meters_per_seconds(thymio['motor.right.speed'])

        # Update estimated position and error
        meanSpeed = (MOT_L_SPEED + MOT_R_SPEED) / 2
        position += DT * meanSpeed
        error = distance - position

        # Compute and apply control input
        if error > ERROR_TOLERANCE:
            speed = constants.MAX_SPEED_METERS_PER_SECOND
            thymio['motor.left.target'] = convert.to_thymio_speed(speed)
            thymio['motor.right.target'] = convert.to_thymio_speed(speed)

        # Stop robot if within error tolerance
        else:
            stop(thymio)
            return

        # Pace loop
        time.sleep(DT)

def turn_angle(thymio: Thymio, radians: float) -> None:

    # Control loop
    ERROR_TOLERANCE = 0.005
    distance = np.abs(radians * constants.WHEELS_PITCH_METERS / 2)
    position = 0
    while True:

        # Read and convert sensors values
        PROX_H      = np.array([convert.to_normalized_proximity(value) for value in thymio['prox.horizontal']])
        ACC         = np.array([convert.to_meters_per_seconds_squared(axis) for axis in thymio['acc']])
        MOT_L_SPEED = convert.to_meters_per_seconds(thymio['motor.left.speed'])
        MOT_R_SPEED = convert.to_meters_per_seconds(thymio['motor.right.speed'])

        # Update estimated position and error
        meanSpeed = (np.abs(MOT_L_SPEED) + np.abs(MOT_R_SPEED)) / 2
        position += DT * meanSpeed
        error = distance - position

        # Compute and apply control input
        if error > ERROR_TOLERANCE:
            speed = constants.MAX_SPEED_METERS_PER_SECOND
            thymio['motor.left.target'] = convert.to_thymio_speed(-speed * np.sign(radians)) 
            thymio['motor.right.target'] = convert.to_thymio_speed(speed * np.sign(radians))

        # Stop robot if within error tolerance
        else:
            stop(thymio)
            return

        # Pace loop
        time.sleep(DT)

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

"""
    with Thymio.serial(port=PORT, refreshing_rate=DT) as thymio:
        print('Starting path following, press CTRL + C to cancel')
        try:
            print('Connecting to Thymio')
            wait_connection(thymio)
            print('Start path following')
            follow_path(thymio, PATH)
        except KeyboardInterrupt:
            pass
        finally:
            print('Ending program')
            stop(thymio)"""
