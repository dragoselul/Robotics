"""
Example usage of RobotKinematics class
Demonstrates various ways to control the robot using kinematics formulas
"""

import sys
import os
import numpy as np

# Add parent directories to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from Robot.robot_kinematics import RobotKinematics


def example_basic_movements():
    """Example 1: Basic movement commands"""
    print("\n" + "="*50)
    print("EXAMPLE 1: Basic Movements")
    print("="*50)
    
    # Initialize robot (adjust device_name for your system)
    # Windows: "COM7", Linux: "/dev/ttyUSB0", Mac: "/dev/tty.usbserial-*"
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Move to home position
        print("\n1. Moving to home position...")
        robot.home_position()
        robot.print_current_state()
        
        # Move to a target XYZ position
        print("\n2. Moving to position [100, 100, 150] mm...")
        target = np.array([100, 100, 150])
        robot.move_to_position(target, elbow_up=True, speed=150)
        robot.print_current_state()
        
    finally:
        robot.close()


def example_trajectory():
    """Example 2: Move along a trajectory"""
    print("\n" + "="*50)
    print("EXAMPLE 2: Trajectory Following")
    print("="*50)
    
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Define a series of waypoints
        waypoints = [
            np.array([100, 0, 100]),
            np.array([100, 50, 120]),
            np.array([100, 100, 140]),
            np.array([100, 100, 160]),
            np.array([80, 100, 160]),
            np.array([60, 100, 140]),
        ]
        
        print(f"\nFollowing trajectory with {len(waypoints)} waypoints...")
        for i, waypoint in enumerate(waypoints):
            print(f"\nWaypoint {i+1}: {waypoint}")
            robot.move_to_position(waypoint, elbow_up=True, speed=100)
            robot.print_current_state()
            
    finally:
        robot.close()


def example_circular_path():
    """Example 3: Draw a circle in 3D space"""
    print("\n" + "="*50)
    print("EXAMPLE 3: Circular Path")
    print("="*50)
    
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Circle parameters
        center = np.array([100, 0, 150])  # Circle center
        radius = 30  # Circle radius in mm
        num_points = 12  # Number of points around circle
        
        print(f"\nDrawing circle: center={center}, radius={radius}mm")
        
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            # Calculate point on circle in YZ plane
            y_offset = radius * np.cos(angle)
            z_offset = radius * np.sin(angle)
            target = center + np.array([0, y_offset, z_offset])
            
            print(f"\nPoint {i+1}/{num_points}: {target}")
            robot.move_to_position(target, elbow_up=True, speed=150)
            
    finally:
        robot.close()


def example_joint_space_control():
    """Example 4: Direct joint angle control"""
    print("\n" + "="*50)
    print("EXAMPLE 4: Joint Space Control")
    print("="*50)
    
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Define joint angles directly (in radians)
        joint_configurations = [
            np.array([0, 0, 0, 0]),                    # Home
            np.array([np.pi/4, 0, 0, 0]),              # Rotate base 45°
            np.array([np.pi/4, np.pi/6, 0, 0]),        # Lift shoulder
            np.array([np.pi/4, np.pi/6, np.pi/4, 0]),  # Bend elbow
            np.array([np.pi/4, np.pi/6, np.pi/4, np.pi/2]),  # Rotate wrist
        ]
        
        print("\nMoving through joint configurations...")
        for i, config in enumerate(joint_configurations):
            print(f"\nConfiguration {i+1}: {np.degrees(config)} degrees")
            robot.move_to_joint_angles(config, speed=100)
            robot.print_current_state()
            
    finally:
        robot.close()


def example_pose_control():
    """Example 5: Control position and orientation"""
    print("\n" + "="*50)
    print("EXAMPLE 5: Full Pose Control (Position + Orientation)")
    print("="*50)
    
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Move to position with specific end-effector orientation
        target_position = np.array([120, 80, 160])
        
        # Try different orientations
        orientations = [
            (np.array([1, 0, 0]), "Point in +X direction"),
            (np.array([0, 1, 0]), "Point in +Y direction"),
            (np.array([0, 0, 1]), "Point in +Z direction"),
            (np.array([1, 1, 0]) / np.sqrt(2), "Point diagonally XY"),
        ]
        
        for orientation, description in orientations:
            print(f"\n{description}")
            print(f"Target position: {target_position}")
            print(f"Target orientation: {orientation}")
            
            try:
                robot.move_to_pose(target_position, orientation, speed=150)
                robot.print_current_state()
            except Exception as e:
                print(f"Failed: {e}")
                
    finally:
        robot.close()


def example_workspace_checking():
    """Example 6: Check workspace reachability"""
    print("\n" + "="*50)
    print("EXAMPLE 6: Workspace Validation")
    print("="*50)
    
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Test various positions
        test_positions = [
            np.array([100, 100, 150]),   # Should be reachable
            np.array([200, 0, 100]),     # Should be reachable
            np.array([500, 500, 500]),   # Too far - unreachable
            np.array([10, 10, 10]),      # Too close - unreachable
        ]
        
        print("\nTesting workspace reachability...")
        for pos in test_positions:
            reachable = robot.is_position_reachable(pos)
            status = "✓ Reachable" if reachable else "✗ Unreachable"
            print(f"Position {pos}: {status}")
            
            if reachable:
                print("  Attempting to move...")
                try:
                    robot.move_to_position(pos, speed=150)
                    print("  Success!")
                except Exception as e:
                    print(f"  Failed: {e}")
                    
    finally:
        robot.close()


def example_pick_and_place():
    """Example 7: Simple pick and place operation"""
    print("\n" + "="*50)
    print("EXAMPLE 7: Pick and Place")
    print("="*50)
    
    robot = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot.initialize()
    
    try:
        # Define pick and place positions
        approach_height = 200  # mm above target
        pick_position = np.array([100, 100, 100])
        place_position = np.array([150, 50, 100])
        
        print("\n=== Pick and Place Sequence ===")
        
        # 1. Move to approach position above pick
        print("\n1. Approaching pick position...")
        approach_pick = pick_position.copy()
        approach_pick[2] = approach_height
        robot.move_to_position(approach_pick, speed=100)
        
        # 2. Move down to pick
        print("\n2. Moving to pick position...")
        robot.move_to_position(pick_position, speed=50)
        print("   [Gripper would close here]")
        
        # 3. Lift up
        print("\n3. Lifting object...")
        robot.move_to_position(approach_pick, speed=50)
        
        # 4. Move to approach position above place
        print("\n4. Moving to place approach...")
        approach_place = place_position.copy()
        approach_place[2] = approach_height
        robot.move_to_position(approach_place, speed=100)
        
        # 5. Move down to place
        print("\n5. Placing object...")
        robot.move_to_position(place_position, speed=50)
        print("   [Gripper would open here]")
        
        # 6. Lift up
        print("\n6. Retracting...")
        robot.move_to_position(approach_place, speed=50)
        
        # 7. Return home
        print("\n7. Returning home...")
        robot.home_position()
        
        print("\n=== Pick and Place Complete! ===")
        
    finally:
        robot.close()


if __name__ == "__main__":
    print("="*60)
    print("Robot Kinematics Examples")
    print("="*60)
    print("\nAvailable examples:")
    print("1. Basic movements")
    print("2. Trajectory following")
    print("3. Circular path")
    print("4. Joint space control")
    print("5. Pose control (position + orientation)")
    print("6. Workspace validation")
    print("7. Pick and place")
    print("\nUncomment the example you want to run in the code.")
    print("="*60)
    
    # Uncomment ONE example to run:
    
    # example_basic_movements()
    # example_trajectory()
    # example_circular_path()
    # example_joint_space_control()
    # example_pose_control()
    # example_workspace_checking()
    # example_pick_and_place()
    
    print("\nNote: Make sure to set the correct device_name for your system!")
    print("  - Windows: 'COM7'")
    print("  - Linux: '/dev/ttyUSB0'")
    print("  - Mac: '/dev/tty.usbserial-*'")
