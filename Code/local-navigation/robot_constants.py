# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Robot calibration constants

# Calibration settings
MAX_PROX_VALUE                      = 4300
MAX_SPEED_METERS_PER_SECOND         = 0.2
METERS_PER_SECONDS_PER_LSB          = MAX_SPEED_METERS_PER_SECOND / 500
METERS_PER_SECONDS_SQUARED_PER_G    = 9.81
G_PER_LSB                           = 1 / 23
WHEELS_PITCH_METERS                 = 0.06

MAX_SPEED_MILLIMETERS_PER_SECOND    = 200
MAX_SPEED_LSB                       = 500
MILLIMETERS_PER_SECOND_PER_LSB      = MAX_SPEED_MILLIMETERS_PER_SECOND / MAX_SPEED_LSB
