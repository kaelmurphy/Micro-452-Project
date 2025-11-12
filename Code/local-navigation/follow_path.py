from Thymio import Thymio
import numpy as np
import time

# Connection settings
PORT            = 'COM5'    # use tty on unix
REFRESH_PERIOD  = 0.1       # seconds

# Conversion constants
MAX_PROX_VALUE  = 4300
M_PER_S_PER_LSB = 0.020 / 500
M_PER_S_2_PER_G = 9.81
G_PER_LSB       = 1 / 23

# Conversion functions

def normalize_prox(raw: int) -> float:
    return raw / MAX_PROX_VALUE

def uint16_to_int16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value

def convert_speed(raw: int) -> float:
    return uint16_to_int16(raw) * M_PER_S_PER_LSB

def convert_acc(raw: int) -> float:
    return uint16_to_int16(raw) * G_PER_LSB * M_PER_S_2_PER_G

# Control function
def follow_path(path: np.ndarray):

    # Open connection with Thymio
    with Thymio.serial(port=PORT, refreshing_rate=REFRESH_PERIOD) as thymio:

        # Wait until first sensor measurment
        time.sleep(REFRESH_PERIOD)
        while thymio['prox.horizontal'] == []:
            time.sleep(REFRESH_PERIOD)

        # Control loop
        i = 20
        position = (0, 0)
        while i:

            # Read and convert sensors values
            PROX_H      = np.array([normalize_prox(value) for value in thymio['prox.horizontal']])
            ACC         = np.array([convert_acc(axis) for axis in thymio['acc']])
            MOT_L_SPEED = convert_speed(thymio['motor.left.speed'])
            MOT_R_SPEED = convert_speed(thymio['motor.right.speed'])

            # Print sensors
            print(PROX_H)
            print(ACC)
            print(MOT_L_SPEED)
            print(MOT_R_SPEED)

            # Pace loop
            time.sleep(REFRESH_PERIOD)
            i -= 1

# Demo program
if __name__ == '__main__':

    # Path coordinates list (in meters)
    PATH = np.array([
        [0.2, 0.0]
    ])

    # Run control routine
    follow_path(PATH)
