import os
import time
from .translation import *
from Assignment.Code.camera.camera import CameraInput
from .robot import Robot

if __name__ == '__main__':
    robot = Robot(device_name='/dev/ttyACM0', baudrate=1_000_000, dxl_ids=[1,2,3,4])
    try:
        robot.initialize()
        # robot.ping_motors()
        time.sleep(1)
        robot.enable_torque([1,2,3,4])
        movement: dict[int,int] = {
            1: deg_to_robot_movement(150),
            2: deg_to_robot_movement(150-30),
            3: deg_to_robot_movement(150),
            4: deg_to_robot_movement(150),
        }
        robot.move(positions=movement)
        camera = CameraInput(0)
        camera.read()
        robot.disable_torque([1,2,3,4])
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        robot.close()
        print("Robot connection closed")
