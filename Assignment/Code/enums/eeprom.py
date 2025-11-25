from enum import Enum

class EepromControlAddresses(Enum):
    MODEL_NUMBER = 0            #R
    FIRMWARE_VERSION = 2        #R
    MOTOR_ID = 3                #RW
    BAUD_RATE = 4               #RW
    RETURN_DELAY_TIME = 5       #RW
    CW_ANGLE_LIMIT = 6          #RW
    CCW_ANGLE_LIMIT = 8         #RW
    TEMPERATURE_LIMIT = 11      #RW
    MIN_VOLTAGE_LIMIT = 12      #RW
    MAX_VOLTAGE_LIMIT = 13      #RW
    MAX_TORQUE = 14             #RW
    STATUS_RETURN_LEVEL = 16    #RW
    ALARM_LED = 17              #RW
    SHUTDOWN = 18               #RW

