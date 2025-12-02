import numpy as np

PER_UNIT_VALUE = 0.29

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

def angles_to_motor_positions(joint_angles):
        """Convert [q1,q2,q3,q4] array to {1:pos, 2:pos...} dict."""
        motor_positions = {}
        for i, angle in enumerate(joint_angles):
            motor_id = i + 1
            val = rad_to_motor_units(angle)
            motor_positions[motor_id] = 512 + val 
        return print(motor_positions)

def motor_units_to_angles(positions_dict):
    """Convert {1:pos, 2:pos...} dict to [q1,q2,q3,q4] array."""
    angles = np.zeros(4)
    for motor_id, pos in positions_dict.items():
        idx = motor_id - 1
        if 0 <= idx < 4:
            angles[idx] = motor_units_to_rad(pos - 512)
    return print(angles)
    


(angles_to_motor_positions(
np.array([ 0.        , -0.43816168, -1.23187902,  0.09924437])))

motor_units_to_angles({1: 512, 2: 425, 3: 269, 4: 532})