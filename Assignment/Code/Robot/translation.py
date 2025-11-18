import numpy as np

PER_UNIT_VALUE = 0.29 # degrees

def deg_to_robot_movement(deg):
    """Convert degrees to robot movement units."""
    return int(deg / PER_UNIT_VALUE)

def robot_movement_to_deg(movement):
    """Convert robot movement units to degrees."""
    return int(movement * PER_UNIT_VALUE)

def deg_to_rad(deg):
    """Convert degrees to radians."""
    return deg * (np.pi / 180)

def rad_to_deg(rad):
    """Convert radians to degrees."""
    return rad * (180 / np.pi)

def rad_to_motor_movement(rad):
    """Convert radians to motor movement units (integer)."""
    deg = rad * (180 / np.pi)  # radians to degrees
    movement = int(deg / PER_UNIT_VALUE)
    return movement

def motor_movement_to_rad(movement):
    """Convert motor movement units (integer) to radians."""
    deg = movement * PER_UNIT_VALUE  # degrees
    rad = deg * (np.pi / 180)        # degrees to radians
    return rad