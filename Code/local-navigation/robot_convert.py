# Author    : Killian Baillifard
# Date      : 12.11.2025
# Brief     : Helper functions for conversion of sensors readings into physical values

# Imports
import robot_constants as constants

# Conversion functions

def to_normalized_proximity(raw: int) -> float:
    """
    Normalize raw proximity sensor value between 0 and 1 (min and max)
    """
    return raw / constants.MAX_PROX_VALUE

def to_int16(unsigned: int) -> int:
    """
    Convert a raw integer into a 16 bit signed integer
    """
    return unsigned - 0x10000 if unsigned & 0x8000 else unsigned

def to_uint16(signed: int) -> int:
    """
    Convert a 16 bit signed integer into an unsigned integer
    """
    return signed + 0x10000 if signed < 0 else signed

def to_meters_per_seconds_squared(raw: int) -> float:
    """
    Convert raw acceleration integer into a float in meters per seconds squared
    """
    return to_int16(raw) * constants.G_PER_LSB * constants.METERS_PER_SECONDS_SQUARED_PER_G

def to_meters_per_seconds(raw: int) -> float:
    """
    Convert raw speed integer into a float in meters per seconds
    """
    return to_int16(raw) * constants.METERS_PER_SECONDS_PER_LSB

def to_thymio_speed(speed: float) -> int:
    """
    Convert a speed in meters per seconds into an integer value for robot target speed
    """
    return to_uint16(int(speed / constants.METERS_PER_SECONDS_PER_LSB))
