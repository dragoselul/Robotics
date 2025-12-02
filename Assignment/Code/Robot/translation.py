import numpy as np

PER_UNIT_VALUE = 0.29 # degrees

def rad_to_motor_units(rad):
    """Convert radians to motor movement units (integer)."""
    deg = rad * (180 / np.pi)  # radians to degrees
    # Round to nearest whole motor unit to keep conversions invertible
    movement = int(np.round(deg / PER_UNIT_VALUE))
    return movement

def motor_units_to_rad(movement):
    """Convert motor movement units (integer) to radians."""
    deg = movement * PER_UNIT_VALUE  # degrees
    rad = deg * (np.pi / 180)        # degrees to radians
    return float(rad)
