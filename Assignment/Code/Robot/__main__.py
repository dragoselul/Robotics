import os
import time
from robot import Robot

def terminal_type():
    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
        return getch
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        return getch

if __name__ == '__main__':
    getch = terminal_type()
    robot = Robot(device_name='COM7', baudrate=1_000_000, dxl_ids=[1,2,3,4])
    robot.open_com()
    robot.ping_motors()
    # robot.enable_torque([1,2])
    print(robot.read_current_position([1,2]))
    robot.move(1, 250)
    robot.move(2, 600)
    # robot.disable_torque([1,2])
    robot.close_com()
