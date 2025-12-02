import time

try:
    from dynamixel_sdk import *  # type: ignore
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    # The SDK isn't installed (e.g., when running in mock mode).
    DYNAMIXEL_AVAILABLE = False

from enums import *


class RobotControls:
    def __init__(self, device_name='/dev/tty.usbmodem14401', baudrate=1_000_000):
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.device_name = device_name
        self.baudrate = baudrate
        self.packetHandler = None
        self.portHandler = None
        if not DYNAMIXEL_AVAILABLE:
            raise ImportError("dynamixel_sdk is not available. Use MockRobotControls or install the SDK.")

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

    def set_goal_position(self, motor_id, goal_position):
        """
        Set the goal position for a motor without waiting.
        """
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id,
                                                                       RAMControlAddresses.GOAL_POSITION.value, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def move(self, motor_id, goal_position, margin_of_error):
        self.set_goal_position(motor_id, goal_position)
        # while 1:
        #     # Read present position
        #     # time.sleep(0.1) # Reduced sleep time
        #     position = self.read_current_position(motor_id)
        #     # print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (motor_id, goal_position, position))
        
        #     if not (abs(goal_position - position) > margin_of_error):
        #         break

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


class MockRobotControls:
    """
    Lightweight stand-in for RobotControls that avoids any hardware I/O.
    Stores motor state in-memory so higher-level code can run without a serial port.
    """

    def __init__(self, device_name=None, baudrate=None, initial_position=512):
        self.device_name = device_name or "mock-port"
        self.baudrate = baudrate or 0
        self.packetHandler = None
        self.portHandler = None
        self._positions = {}
        self._speeds = {}
        self._initial_position = initial_position

    def open_com(self):
        print(f"[MOCK] Opening dummy connection on {self.device_name} @ {self.baudrate} baud")

    def close_com(self):
        print("[MOCK] Closing dummy connection")

    def torque_enable_control(self, motor_id, enable=False):
        print(f"[MOCK] Torque {'EN' if enable else 'DIS'} for motor {motor_id}")
        return True

    def ping_motor(self, motor_id):
        print(f"[MOCK] Ping motor {motor_id}: OK")

    def set_moving_speed(self, motor_id, speed):
        assert speed >= 0 and speed <= 1023, "Speed must be between 0 and 1023"
        self._speeds[motor_id] = speed
        print(f"[MOCK] Set speed {speed} for motor {motor_id}")
        return True

    def set_torque_limit(self, motor_id, torque):
        assert torque >= 0 and torque <= 1023, "Torque must be between 0 and 1023"
        print(f"[MOCK] Set torque limit {torque} for motor {motor_id}")
        return True

    def set_goal_position(self, motor_id, goal_position):
        self._positions[motor_id] = goal_position
        print(f"[MOCK] Set goal position {goal_position} for motor {motor_id}")

    def move(self, motor_id, goal_position, margin_of_error):
        # Simulate immediate movement
        self.set_goal_position(motor_id, goal_position)
        time.sleep(0.01)

    def read_current_position(self, motor_id):
        return self._positions.get(motor_id, self._initial_position)

    def read_current_speed(self, motor_id):
        return self._speeds.get(motor_id, 0)
