import os
import time

from Assignment.Code.camera.camera import CameraInput
from robot import Robot

if __name__ == '__main__':
    robot = Robot(device_name='/dev/ttyACM0', baudrate=1_000_000, dxl_ids=[1,2,3,4])
    robot.initialize()
    robot.ping_motors()
    

    camera = CameraInput(2)
    camera.read()
    robot.disable_torque([1,2,3,4])
    robot.close()