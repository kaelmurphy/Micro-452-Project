# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Implementation of local navigation from a given path to follow

# Imports
from Thymio import Thymio
import numpy as np
import robot_convert as convert
from robot_control import control_law
import time

# Connection settings
PORT    = 'COM3'    # use tty on unix
DT      = 0.1       # seconds

# Control routines
def wait_connection(thymio: Thymio) -> None:

    # Wait until last key is created
    TEST_SENSOR = 'sd.present'
    while True:
        try:
            _ = thymio[TEST_SENSOR]
            break
        except KeyError:
            pass
        finally:
            time.sleep(DT)

    # Wait until last key is readable
    while thymio[TEST_SENSOR] == []:
        time.sleep(DT)

def follow_path(thymio: Thymio, path: np.ndarray):

    # Control loop
    position = np.array([0, 0])
    while True:

        # Read and convert sensors values
        PROX_H      = np.array([convert.to_norm_prox(value) for value in thymio['prox.horizontal']])
        ACC         = np.array([convert.to_mps2(axis) for axis in thymio['acc']])
        MOT_L_SPEED = convert.to_mps(thymio['motor.left.speed'])
        MOT_R_SPEED = convert.to_mps(thymio['motor.right.speed'])

        # Update estimated position
        meanSpeed = (MOT_L_SPEED + MOT_R_SPEED) / 2
        position = np.array([
            position[0] + DT * meanSpeed,
            position[1]
        ])

        # Compute error, stop if error is less that 1 cm
        error = path[0][0] - position[0]
        if error < 0.01:
            break

        # Compute and apply control input
        speed = control_law(error)
        thymio['motor.left.target'] = convert.to_int_speed(speed)
        thymio['motor.right.target'] = convert.to_int_speed(speed)

        # Pace loop
        time.sleep(DT)

def stop(thymio: Thymio):

    # Give stop command
    thymio['motor.left.target'] = 0
    thymio['motor.right.target'] = 0

    # Wait until speed fall under threshold
    SPEED_THRESHOLD = 0.001
    while True:
        time.sleep(DT)
        MOT_L_SPEED = convert.to_mps(thymio['motor.left.speed'])
        MOT_R_SPEED = convert.to_mps(thymio['motor.right.speed'])
        if MOT_L_SPEED < SPEED_THRESHOLD and MOT_R_SPEED < SPEED_THRESHOLD:
            return

if __name__ == '__main__':

    # Path coordinates list (in meters)
    PATH = np.array([
        [0.1, 0.0]
    ])

    with Thymio.serial(port=PORT, refreshing_rate=DT) as thymio:
        print('Starting path following, press CTRL + C to cancel')
        try:
            print('Connecting to Thymio ... ', end='')
            wait_connection(thymio)
            print('Connected')
            print('Start following path')
            follow_path(thymio, PATH)
        except KeyboardInterrupt:
            pass
        finally:
            print('Cutting off motors')
            stop(thymio)
