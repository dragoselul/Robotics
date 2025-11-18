from .robot_controls import RobotControls
import threading


class Robot:

    def __init__(self, device_name="COM7", baudrate=1_000_000, dxl_ids=[1, 2, 3, 4]):
        self.dxl_ids = dxl_ids
        self.robot_control = RobotControls(device_name, baudrate)

    def initialize(self):
        self.robot_control.open_com()
        self.enable_torque([1, 2, 3, 4])
        self.set_move_speed([1, 2, 3, 4], 150)
        self.move({1: 512, 2: 512, 3: 512, 4: 512})
        self.disable_torque([1, 2, 3, 4])

    def enable_torque(self, dxl_ids=[]):
        for dxl_id in dxl_ids:
            self.robot_control.torque_enable_control(dxl_id, True)

    def disable_torque(self, dxl_ids=[]):
        for dxl_id in dxl_ids:
            self.robot_control.torque_enable_control(dxl_id, False)

    def ping_motors(self):
        for dxl_id in self.dxl_ids:
            self.robot_control.ping_motor(dxl_id)

    def read_current_position(self, dxl_ids=[]):
        positions = {}
        for dxl_id in dxl_ids:
            positions[dxl_id] = self.robot_control.read_current_position(dxl_id)
        return positions

    def set_move_speed(self, dxl_ids=[], speed=512):
        for dxl_id in dxl_ids:
            self.robot_control.set_moving_speed(dxl_id, speed)


    def move(self, positions: dict[int, int], margin_of_error=10):
        for motor_id, goal_position in positions.items():
            match motor_id:
                case 1:
                    if goal_position < 200:
                        goal_position = 200
                    elif goal_position > 900:
                        goal_position = 900
                    self.robot_control.move(motor_id, goal_position, margin_of_error)
                case 2:
                    if goal_position < 200:
                        goal_position = 200
                    elif goal_position > 800:
                        goal_position = 800
                    elif goal_position < 375:
                        positions = self.read_current_position([3,4])
                        if positions[3] < 500:
                            goal_position = 300
                        elif positions[3] < 400:
                            goal_position = 375
                        elif positions[4] < 600:
                            goal_position = 375
                    self.robot_control.move(motor_id, goal_position, margin_of_error)
                case 3:
                    if goal_position < 40:
                        goal_position = 40
                    elif goal_position > 900:
                        goal_position = 900
                    self.robot_control.move(motor_id, goal_position, margin_of_error)
                case 4:
                    if goal_position < 500:
                        goal_position = 500
                    elif goal_position > 1000:
                        goal_position = 1000
                    self.robot_control.move(motor_id, goal_position, margin_of_error)
                case _:
                    raise ValueError("Invalid dxl_id")

    def close(self):
        self.robot_control.close_com()
