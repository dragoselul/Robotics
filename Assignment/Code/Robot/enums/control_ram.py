from enum import Enum

class RAMControlAddresses(Enum):
    TORQUE_ENABLE = 24          # RW
    LED_ENABLE = 25             # RW
    CW_COMPLIANCE_MARGIN = 26   # RW
    CCW_COMPLIANCE_MARGIN = 27  # RW
    CW_COMPLIANCE_SLOPE = 28    # RW
    CCW_COMPLIANCE_SLOPE = 29   # RW
    GOAL_POSITION = 30          # RW
    MOVING_SPEED = 32           # RW
    TORQUE_LIMIT = 34           # RW
    PRESENT_POSITION = 36       # R
    PRESENT_SPEED = 38          # R
    PRESENT_LOAD = 40           # R
    PRESENT_VOLTAGE = 42        # R
    PRESENT_TEMPERATURE = 43    # R
    REGISTERED = 44             # R
    MOVING = 46                 # R
    LOCK = 47                   # RW
    PUNCH = 48                  # RW