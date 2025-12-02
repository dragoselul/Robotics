try:
    from .robot_controller import RobotKinematicsController 
    from .robot import Robot
except ImportError:
    from robot_controller import RobotKinematicsController
    from robot import Robot

from Dragos.Part1_Problem_3 import execute_circle_movement




def main():
    hardware = Robot(device_name='/dev/ttyACM0', baudrate=1_000_000, dxl_ids=[1, 2, 3, 4]) 
    controller = RobotKinematicsController(hardware, verbose=True)
    controller.initialize()

    execute_circle_movement(controller=controller, center=[150, 0, 120], radius=32, speed=500)

if __name__ == "__main__":
    main()
