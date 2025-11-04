from dynamixel_sdk import *

ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10                            # Dynamixel moving status threshold
COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed
PROTOCOL_VERSION            = 1.0

class Robot:
    
    def __init__(self, device_name='COM7', baudrate=1_000_000, dxl_ids=[1,2,3,4]):
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.device_name = device_name
        self.baudrate = baudrate
        self.dxl_ids = dxl_ids
        self.packetHandler = None
        self.portHandler = None

    def initialize(self):
        self.enable_torque(self.dxl_ids)

        for motor in self.dxl_ids:
            self.move(motor, 0)

        self.disable_torque(self.dxl_ids)
        
        
    def open_com(self):
        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.device_name)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Set port baudrate which apparently opens the port as well
        if not self.portHandler.setBaudRate(self.baudrate):
            print("Failed to change the baudrate")

        print(self.portHandler.is_open)
        # if self.portHandler.is_open:
        #     self.initialize()

    def enable_torque(self, dxl_ids=[]):
        for dxl_id in dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel has been successfully connected")

    def disable_torque(self, dxl_ids=[]):
        for dxl_id in dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def close_com(self):
        self.portHandler.closePort()

    def ping_motors(self):
        for dxl_id in self.dxl_ids:
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, dxl_id)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (dxl_id, dxl_model_number))

    def read_current_position(self, dxl_ids=[]):
        positions = {}
        for dxl_id in dxl_ids:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            positions[dxl_id] = dxl_present_position
        return positions

    def move(self, dxl_id, goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        while 1:
        # Read present position
            positions = self.read_current_position([dxl_id])
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (dxl_id, goal_position, positions[dxl_id]))

            if not (abs(goal_position - positions[dxl_id]) > DXL_MOVING_STATUS_THRESHOLD):
                break
