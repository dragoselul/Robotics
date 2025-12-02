import numpy as np

PER_UNIT_VALUE = 0.29 # degrees

def rad_to_motor_units(rad):
    deg = rad * (180 / np.pi)  # radians to degrees
    try:
        movement = int(np.round(deg / PER_UNIT_VALUE))
    except Warning:
        print("Warning in rad_to_motor_units with value:", rad)
        movement = int(np.round(deg / PER_UNIT_VALUE))
    return movement

def motor_units_to_rad(movement):
    deg = movement * PER_UNIT_VALUE  # degrees
    rad = deg * (np.pi / 180)        # degrees to radians
    return float(rad)
