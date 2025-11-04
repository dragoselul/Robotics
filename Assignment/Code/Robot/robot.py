from Assignment.Code.Robot.robot_controls import RobotControls


class Robot:

    def __init__(self, device_name="COM7", baudrate=1_000_000, dxl_ids=[1, 2, 3, 4]):
        self.dxl_ids = dxl_ids
        self.robot_control = RobotControls(device_name, baudrate)

    def initialize(self):
        self.robot_control.open_com()
        self.enable_torque([1, 2, 3, 4])
        self.set_move_speed([1, 2, 3, 4], 150)
        self.move(1, 400, margin_of_error=30)
        self.move(2, 600)
        self.move(3, 400)
        self.move(4, 200)
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


    def move(self, dxl_id, goal_position, margin_of_error=10):
        match dxl_id:
            case 1:
                assert 300 <= goal_position <= 900, "Goal position for motor 1 must be between 0 and 1023"
                self.robot_control.move(dxl_id, goal_position, margin_of_error)
            case 2:
                assert 200 <= goal_position <= 800, "Goal position for motor 2 must be between 0 and 1023"
                self.robot_control.move(dxl_id, goal_position, margin_of_error)
            case 3:
                assert 40 <= goal_position <= 800, "Goal position for motor 3 must be between 0 and 1023"
                self.robot_control.move(dxl_id, goal_position, margin_of_error)
            case 4:
                assert 150 <= goal_position <= 750, "Goal position for motor 4 must be between 0 and 1023"
                self.robot_control.move(dxl_id, goal_position, margin_of_error)
            case _:
                raise ValueError("Invalid dxl_id")

    def close(self):
        self.robot_control.close_com()
