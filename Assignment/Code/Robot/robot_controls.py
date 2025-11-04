from dynamixel_sdk import *

from Assignment.Code.enums import *


class RobotControls:
    def __init__(self, device_name='COM7', baudrate=1_000_000):
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.device_name = device_name
        self.baudrate = baudrate
        self.packetHandler = None
        self.portHandler = None

    def open_com(self):
        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.device_name)
        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(protocol_version=1.0)

        # Set port baudrate which apparently opens the port as well
        if not self.portHandler.setBaudRate(self.baudrate):
            print("Failed to change the baudrate")

        print(self.portHandler.is_open)

    def close_com(self):
        self.portHandler.closePort()

    def torque_enable_control(self, motor_id, enable=False):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id,
                                                                       RAMControlAddresses.TORQUE_ENABLE.value, enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            if enable:
                self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, RAMControlAddresses.LED_ENABLE.value, 1)
            else:
                self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, RAMControlAddresses.LED_ENABLE.value, 0)
            return True

    def ping_motor(self, motor_id):
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, motor_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (motor_id, dxl_model_number))

    def set_moving_speed(self, motor_id, speed):
        assert speed >= 0 and speed <= 1023, "Speed must be between 0 and 1023"
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id,
                                                                       RAMControlAddresses.MOVING_SPEED.value, speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        return True

    def set_torque_limit(self, motor_id, torque):
        assert torque >= 0 and torque <= 1023, "Torque must be between 0 and 1023"
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id,
                                                                       RAMControlAddresses.TORQUE_LIMIT.value, torque)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        return True

    def move(self, motor_id, goal_position, margin_of_error):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id,
                                                                       RAMControlAddresses.GOAL_POSITION.value, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        while 1:
            # Read present position
            position = self.read_current_position(motor_id)
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (motor_id, goal_position, position))

            if not (abs(goal_position - position) > margin_of_error):
                break

    def read_current_position(self, motor_id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id,
                                                                                            RAMControlAddresses.PRESENT_POSITION.value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_position

    def read_current_speed(self, motor_id):
        dxl_present_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id,
                                                                                         RAMControlAddresses.PRESENT_SPEED.value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_speed
