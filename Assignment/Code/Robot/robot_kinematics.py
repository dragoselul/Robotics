"""
Robot Kinematics Controller
Bridges mathematical kinematics formulas with physical robot control.
Uses numpy-based kinematics functions to compute joint angles and moves the robot accordingly.
"""

import numpy as np
from ..util.AssignmentFunctions import *
from .robot import Robot
from .translation import *


class RobotKinematics:
    """
    High-level robot controller that uses kinematics to move the robot.
    
    Design choices:
    1. Separates mathematical calculations from hardware control
    2. Handles angle-to-motor position conversion
    3. Provides both forward and inverse kinematics methods
    4. Includes safety checks and position validation
    5. Supports both joint-space and task-space control
    """
    
    # Robot DH parameters (in mm and radians)
    LINK_LENGTHS = {
        'L1': 50,   # Base height
        'L2': 93,   # Upper arm
        'L3': 93,   # Forearm
        'L4': 50    # End effector
    }
    
    # Motor position limits (in motor units: 0-1023)
    MOTOR_LIMITS = {
        1: (200, 900),   # Base rotation
        2: (200, 800),   # Shoulder
        3: (40, 900),    # Elbow
        4: (500, 1000)   # Wrist
    }
    
    # Conversion factors
    MOTOR_CENTER = 512  # Center position in motor units
    ANGLE_TO_MOTOR = 1024 / (2 * np.pi)  # Convert radians to motor units
    MOTOR_TO_ANGLE = (2 * np.pi) / 1024  # Convert motor units to radians
    
    def __init__(self, device_name="COM7", baudrate=1_000_000, dxl_ids=[1, 2, 3, 4]):
        """
        Initialize the robot kinematics controller.
        
        Args:
            device_name: Serial port for robot communication
            baudrate: Communication baudrate
            dxl_ids: List of Dynamixel motor IDs
        """
        self.robot = Robot(device_name, baudrate, dxl_ids)
        self.dxl_ids = dxl_ids
        self.current_angles = None  # Cache current joint angles
        
    def initialize(self):
        """Initialize robot to home position."""
        self.robot.initialize()
        print("Robot initialized to home position")
        
    def close(self):
        """Safely close robot connection."""
        self.robot.close()
        print("Robot connection closed")
        
    # ==================== Conversion Methods ====================
    
    def angles_to_motor_positions(self, angles, ignore_limits=False):
        """
        Convert joint angles (radians) to motor positions (0-1023).
        
        Args:
            angles: Array of joint angles [theta1, theta2, theta3, theta4] in radians
            
        Returns:
            Dictionary mapping motor_id to position: {1: pos1, 2: pos2, ...}
        """
        positions = {}
        for i, angle in enumerate(angles):
            motor_id = i + 1
            # Convert angle to motor units and add center offset
            position = int(self.MOTOR_CENTER + angle * self.ANGLE_TO_MOTOR)
            
            # Apply motor-specific offsets for mechanical differences
            if motor_id == 2:
                # Motor 2 has a 90° mechanical offset
                # IK returns angles with -π/2 already included, so we add +π/2 for the motor
                position = int(self.MOTOR_CENTER + (angle + np.pi/2) * self.ANGLE_TO_MOTOR)

            # Clamp to motor limits
            if ignore_limits:
                positions[motor_id] = position
                continue
            min_pos, max_pos = self.MOTOR_LIMITS[motor_id]
            position = np.clip(position, min_pos, max_pos)
            positions[motor_id] = position
            
        return positions
    
    def motor_positions_to_angles(self, positions):
        """
        Convert motor positions (0-1023) to joint angles (radians).
        
        Args:
            positions: Dictionary mapping motor_id to position: {1: pos1, 2: pos2, ...}
            
        Returns:
            Array of joint angles [theta1, theta2, theta3, theta4] in radians
        """
        angles = []
        for motor_id in sorted(positions.keys()):
            position = positions[motor_id]
            angle = (position - self.MOTOR_CENTER) * self.MOTOR_TO_ANGLE
            
            # Apply motor-specific offsets for mechanical differences
            if motor_id == 2:
                # Motor 2 has a 90° mechanical offset
                angle -= np.pi/2

            angles.append(angle)
            
        return np.array(angles)
    
    # ==================== Forward Kinematics ====================
    
    def compute_DH_matrices(self, theta1, theta2, theta3, theta4):
        """
        Compute DH transformation matrices for given joint angles.
        
        Args:
            theta1, theta2, theta3, theta4: Joint angles in radians
            
        Returns:
            List of transformation matrices [T01, T12, T23, T34]

        Note:
            The robot's physical joint 2 has a 90° mechanical offset.
            The IK functions (inverse_kinematics0_3) account for this by computing
            theta2 with a -π/2 term. To match the robot hardware, we add +π/2 here.
        """
        T01 = DH(theta1, self.LINK_LENGTHS['L1'], 0, np.pi/2)
        T12 = DH(theta2 + np.pi/2, 0, self.LINK_LENGTHS['L2'], 0)
        T23 = DH(theta3, 0, self.LINK_LENGTHS['L3'], 0)
        T34 = DH(theta4, 0, self.LINK_LENGTHS['L4'], 0)
        return [T01, T12, T23, T34]
    
    def forward_kinematics(self, angles):
        """
        Compute end-effector pose from joint angles.
        
        Args:
            angles: Array of joint angles [theta1, theta2, theta3, theta4] in radians
            
        Returns:
            T04: 4x4 transformation matrix from base to end-effector
        """
        theta1, theta2, theta3, theta4 = angles
        dh_matrices = self.compute_DH_matrices(theta1, theta2, theta3, theta4)
        T04 = forward_kinematics(dh_matrices)
        return T04
    
    def get_current_pose(self):
        """
        Get current end-effector pose from robot's current joint positions.
        
        Returns:
            Tuple of (position, orientation_matrix)
            - position: 3D numpy array [x, y, z]
            - orientation_matrix: 3x3 rotation matrix
        """
        # Read current motor positions
        motor_positions = self.robot.read_current_position(self.dxl_ids)
        
        # Convert to angles
        angles = self.motor_positions_to_angles(motor_positions)
        self.current_angles = angles
        
        # Compute forward kinematics
        T04 = self.forward_kinematics(angles)
        
        position = extract_XYZ(T04)
        orientation = T04[0:3, 0:3]
        
        return position, orientation
    
    # ==================== Inverse Kinematics ====================
    
    def inverse_kinematics_position_3dof(self, target_position, elbow_up=True):
        """
        Compute joint angles to reach a target XYZ position (3-DOF IK).
        
        Note: This computes IK to position the wrist center (frame 3) at target_position.
        The actual end-effector will be L4 (50mm) further along the x4 axis.
        For precise end-effector positioning, use inverse_kinematics_pose instead.

        Args:
            target_position: 3D numpy array [x, y, z] in mm - desired wrist center position
            elbow_up: If True, use elbow-up configuration; else elbow-down
            
        Returns:
            Array of joint angles [theta1, theta2, theta3] in radians
        """
        solutions = inverse_kinematics0_3(target_position)
        
        # Choose configuration
        if elbow_up:
            angles = solutions[1, :]  # Row 1: elbow up
        else:
            angles = solutions[0, :]  # Row 0: elbow down
            
        return angles

    def inverse_kinematics_position_4dof(self, target_position, elbow_up=True):
        solutions = inverse_kinematics4(target_position)

        # Choose configuration
        if elbow_up:
            angles = solutions[1, :]  # Row 1: elbow up
        else:
            angles = solutions[0, :]  # Row 0: elbow down

        return angles
    
    def inverse_kinematics_pose(self, target_position, target_orientation_x):
        """
        Compute joint angles to reach a target pose (position + orientation, 4-DOF IK).
        
        Args:
            target_position: 3D numpy array [x, y, z] in mm
            target_orientation_x: Desired x-axis direction vector (3D numpy array)
            
        Returns:
            Array of joint angles [theta1, theta2, theta3, theta4] in radians
        """
        # Define T03 function for inverse kinematics
        def T03_func(t1, t2, t3):
            T01 = DH(t1, self.LINK_LENGTHS['L1'], 0, np.pi/2)
            T12 = DH(t2 + np.pi/2, 0, self.LINK_LENGTHS['L2'], 0)
            T23 = DH(t3, 0, self.LINK_LENGTHS['L3'], 0)
            return T01 @ T12 @ T23
        
        angles = inverse_kinematics4(target_orientation_x, target_position, T03_func)
        return angles
    
    # ==================== Movement Methods ====================
    
    def move_to_joint_angles(self, angles, speed=150, margin_of_error=10):
        """
        Move robot to specified joint angles.
        
        Args:
            angles: Array of joint angles [theta1, theta2, theta3, theta4] in radians
            speed: Movement speed (0-1023)
            margin_of_error: Acceptable position error in motor units
            
        Returns:
            True if movement successful, False otherwise
        """
        # Convert angles to motor positions
        positions = self.angles_to_motor_positions(angles)
        
        print(f"Moving to angles (rad): {angles}")
        print(f"Motor positions: {positions}")
        
        # Enable torque, set speed, and move
        self.robot.enable_torque(self.dxl_ids)
        self.robot.set_move_speed(self.dxl_ids, speed)
        self.robot.move(positions, margin_of_error)
        
        # Update cached angles
        self.current_angles = angles
        
        return True
    
    def move_to_position(self, target_position, elbow_up=True, speed=150):
        """
        Move end-effector to target XYZ position using inverse kinematics.
        
        Args:
            target_position: 3D numpy array [x, y, z] in mm
            elbow_up: Configuration choice for IK solution
            speed: Movement speed
            
        Returns:
            True if movement successful, False otherwise
        """
        try:
            # Compute inverse kinematics (3-DOF)
            angles_3dof = self.inverse_kinematics_position(target_position, elbow_up)
            
            # Add theta4 = 0 for 4-DOF control
            angles = np.append(angles_3dof, 0)
            
            # Move robot
            return self.move_to_joint_angles(angles, speed)
            
        except Exception as e:
            print(f"Error moving to position: {e}")
            return False
    
    def move_to_pose(self, target_position, target_orientation_x, speed=150):
        """
        Move end-effector to target pose (position + orientation).
        
        Args:
            target_position: 3D numpy array [x, y, z] in mm
            target_orientation_x: Desired x-axis direction vector
            speed: Movement speed
            
        Returns:
            True if movement successful, False otherwise
        """
        try:
            # Compute inverse kinematics (4-DOF)
            angles = self.inverse_kinematics_pose(target_position, target_orientation_x)
            
            # Move robot
            return self.move_to_joint_angles(angles, speed)
            
        except Exception as e:
            print(f"Error moving to pose: {e}")
            return False
    
    # ==================== Utility Methods ====================
    
    def home_position(self):
        """Move robot to home position (all joints at 0 degrees)."""
        home_angles = np.array([512, 512, 512, 512])
        return self.move_to_joint_angles(home_angles, speed=100)
    
    def print_current_state(self):
        """Print current joint angles and end-effector pose."""
        position, orientation = self.get_current_pose()
        print("\n=== Current Robot State ===")
        print(f"Joint Angles (rad): {self.current_angles}")
        if self.current_angles is not None:
            print(f"Joint Angles (deg): {np.degrees(self.current_angles)}")
        print(f"End-Effector Position (mm): {position}")
        print(f"End-Effector Orientation:\n{orientation}")
        print("===========================\n")
        
    def is_position_reachable(self, target_position):
        """
        Check if a target position is within the robot's workspace.
        
        Args:
            target_position: 3D numpy array [x, y, z] in mm
            
        Returns:
            True if reachable, False otherwise
        """
        try:
            # Try to compute inverse kinematics
            inverse_kinematics0_3(target_position)
            return True
        except:
            return False


# ==================== Example Usage ====================

if __name__ == "__main__":
    # Example usage of the RobotKinematics class
    
    # Initialize robot
    robot_kin = RobotKinematics(device_name="/dev/ttyUSB0", baudrate=1_000_000)
    robot_kin.initialize()
    
    try:
        # Example 1: Move to home position
        print("Moving to home position...")
        robot_kin.home_position()
        robot_kin.print_current_state()
        
        # Example 2: Move to a specific XYZ position
        print("\nMoving to target position [100, 100, 150]...")
        target_pos = np.array([100, 100, 150])
        robot_kin.move_to_position(target_pos, elbow_up=True, speed=150)
        robot_kin.print_current_state()
        
        # Example 3: Move to a specific pose with orientation
        print("\nMoving to target pose...")
        target_pos = np.array([120, 80, 160])
        target_x_axis = np.array([1, 0, 0])  # Point in +X direction
        robot_kin.move_to_pose(target_pos, target_x_axis, speed=150)
        robot_kin.print_current_state()
        
        # Example 4: Move to specific joint angles
        print("\nMoving to specific joint angles...")
        angles = np.array([np.pi/4, np.pi/6, np.pi/3, 0])
        robot_kin.move_to_joint_angles(angles, speed=150)
        robot_kin.print_current_state()
        
    finally:
        # Always close the connection
        robot_kin.close()
