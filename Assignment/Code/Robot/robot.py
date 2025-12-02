from robot_controls import RobotControls, MockRobotControls
import threading


class Robot:

    def __init__(self, device_name="/dev/tty.usbmodem14401", baudrate=1_000_000, dxl_ids=[1, 2, 3, 4], mock=False):
        self.dxl_ids = dxl_ids
        self.mock = mock
        controls_cls = MockRobotControls if mock else RobotControls
        self.robot_control = controls_cls(device_name, baudrate)

    def initialize(self):
        self.robot_control.open_com()
        self.enable_torque([1, 2, 3, 4])
        # self.set_move_speed([1, 2, 3, 4], 150)
        # self.disable_torque([1, 2, 3, 4])

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
        target_positions = {}
        
        # 1. Apply limits and safety logic
        for motor_id, raw_goal in positions.items():
            goal_position = raw_goal
            match motor_id:
                case 1:
                    if goal_position < 200:
                        goal_position = 200
                    elif goal_position > 900:
                        goal_position = 900
                case 2:
                    if goal_position < 200:
                        goal_position = 200
                    elif goal_position > 800:
                        goal_position = 800
                    elif goal_position < 375:
                        # Safety check based on other motors
                        current_pos = self.read_current_position([3,4])
                        if current_pos[3] < 500:
                            goal_position = 300
                        elif current_pos[3] < 400:
                            goal_position = 375
                        elif current_pos[4] < 600:
                            goal_position = 375
                case 3:
                    if goal_position < 40:
                        goal_position = 40
                    elif goal_position > 900:
                        goal_position = 900
                case 4:
                    if goal_position < 0:
                        goal_position = 0
                    elif goal_position > 900:
                        goal_position = 900
                case _:
                    raise ValueError("Invalid dxl_id")
            
            target_positions[motor_id] = goal_position

        for motor_id, goal in target_positions.items():
            self.robot_control.set_goal_position(motor_id, goal)

        while True:
            all_reached = True
            # Read all positions
            current_positions = self.read_current_position(list(target_positions.keys()))
            
            for motor_id, goal in target_positions.items():
                current = current_positions[motor_id]
                if abs(goal - current) > margin_of_error:
                    all_reached = False
                    break
            
            if all_reached:
                break

    def close(self):
        self.robot_control.close_com()
