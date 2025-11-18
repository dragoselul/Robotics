"""
Quick test script for RobotKinematics class
Run this to verify the class works with your setup
"""
import sys
import os

# Add the parent directory to the Python path to enable proper imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import numpy as np
from Assignment.Code.Robot.robot_kinematics import RobotKinematics
from Assignment.Code.util.AssignmentFunctions import *


def test_without_hardware():
    """Test kinematics calculations without connecting to hardware"""
    print("="*60)
    print("TEST 1: Kinematics Calculations (No Hardware Required)")
    print("="*60)
    
    # Create instance (won't initialize hardware)
    robot = RobotKinematics()
    
    print("\n--- Testing Forward Kinematics ---")
    # Test forward kinematics with known angles
    test_angles = np.array([0, 0, 0, 0])
    T04 = robot.forward_kinematics(test_angles)
    print(f"Input angles: {test_angles}")
    print(f"End-effector transformation matrix:\n{T04}")
    
    
    position = extract_XYZ(T04)
    print(f"End-effector position: {position}")
    
    print("\n--- Testing Inverse Kinematics (Position Only) ---")
    # Test inverse kinematics
    target_position = np.array([100, 100, 150])
    print(f"Target position (wrist center): {target_position}")

    try:
        angles_3dof = robot.inverse_kinematics_position(target_position, elbow_up=True)
        print(f"Computed joint angles (3-DOF): {angles_3dof}")
        print(f"In degrees: {np.degrees(angles_3dof)}")
        
        # Verify by forward kinematics to wrist center (T03)
        # Note: inverse_kinematics_position targets the wrist center, not end-effector
        from Assignment.Code.util.AssignmentFunctions import DH
        T01 = DH(angles_3dof[0], 50, 0, np.pi/2)
        T12 = DH(angles_3dof[1] + np.pi/2, 0, 93, 0)  # Robot has mechanical offset in joint 2
        T23 = DH(angles_3dof[2], 0, 93, 0)
        T03 = T01 @ T12 @ T23
        computed_wrist_position = extract_XYZ(T03)
        print(f"Verification (FK to wrist center): {computed_wrist_position}")
        error = np.linalg.norm(computed_wrist_position - target_position)
        print(f"Wrist center error: {error:.6f} mm")

        if error < 1.0:  # Allow 1mm tolerance for numerical errors
            print("✓ Inverse kinematics verification PASSED")
        else:
            print("✗ Inverse kinematics verification FAILED")
            
    except Exception as e:
        print(f"✗ Error computing inverse kinematics: {e}")
    
    print("\n--- Testing Inverse Kinematics (End-Effector Pose, 4-DOF) ---")
    # Test 4-DOF inverse kinematics for end-effector position and orientation
    # First, get a feasible pose by doing FK with known angles, then test IK on that pose
    print("Step 1: Generate a feasible target pose using FK")
    test_angles_for_pose = np.array([np.pi/6, -np.pi/4, np.pi/3, np.pi/6])
    print(f"Test angles: {np.degrees(test_angles_for_pose)} degrees")

    T04_target = robot.forward_kinematics(test_angles_for_pose)
    target_ee_position = extract_XYZ(T04_target)
    target_orientation_x = extract_XYZ_vectors(T04_target, "x")

    print(f"Target end-effector position: {target_ee_position}")
    print(f"Target orientation (x-axis): {target_orientation_x}")

    try:
        print("\nStep 2: Compute IK to reach that pose")
        angles_4dof = robot.inverse_kinematics_pose(target_ee_position, target_orientation_x)
        print(f"Computed joint angles (4-DOF): {angles_4dof}")
        print(f"In degrees: {np.degrees(angles_4dof)}")

        print("\nStep 3: Verify by forward kinematics")
        T04_verify = robot.forward_kinematics(angles_4dof)
        computed_ee_position = extract_XYZ(T04_verify)
        computed_orientation_x = extract_XYZ_vectors(T04_verify, "x")

        print(f"Verification (FK to end-effector):")
        print(f"  Position: {computed_ee_position}")
        print(f"  Orientation (x-axis): {computed_orientation_x}")

        position_error = np.linalg.norm(computed_ee_position - target_ee_position)
        orientation_error = np.linalg.norm(computed_orientation_x - target_orientation_x)

        print(f"  Position error: {position_error:.6f} mm")
        print(f"  Orientation error: {orientation_error:.6f}")

        if position_error < 1.0 and orientation_error < 0.01:
            print("✓ 4-DOF Inverse kinematics verification PASSED")
        else:
            print("✗ 4-DOF Inverse kinematics verification FAILED")

    except Exception as e:
        print(f"✗ Error computing 4-DOF inverse kinematics: {e}")
        import traceback
        traceback.print_exc()

    print("\n--- Testing Angle-Motor Conversion ---")
    # Use angles within valid motor range (considering motor 2's limits and offset)
    # Motor 2: limit (200, 800) with +90° offset means valid angle range is roughly -110° to +20°
    test_angles = np.array([np.pi/4, -np.pi/6, np.pi/6, 0])  # [45°, -30°, 30°, 0°]
    print(f"Test angles (rad): {test_angles}")
    print(f"Test angles (deg): {np.degrees(test_angles)}")
    
    motor_positions = robot.angles_to_motor_positions(test_angles)
    print(f"Motor positions: {motor_positions}")
    
    # Check if any positions were clamped
    clamped = []
    for motor_id, pos in motor_positions.items():
        min_pos, max_pos = robot.MOTOR_LIMITS[motor_id]
        if pos <= min_pos or pos >= max_pos:
            clamped.append(f"Motor {motor_id} at limit")
    if clamped:
        print(f"Warning: {', '.join(clamped)}")

    converted_back = robot.motor_positions_to_angles(motor_positions)
    print(f"Converted back (rad): {converted_back}")
    conversion_error = np.linalg.norm(test_angles - converted_back)
    print(f"Conversion error: {conversion_error:.6f} rad")
    print(f"Max angle error: {np.max(np.abs(test_angles - converted_back)):.6f} rad ({np.degrees(np.max(np.abs(test_angles - converted_back))):.3f}°)")

    # Allow reasonable tolerance for integer rounding (but not clamping)
    if conversion_error < 0.01:  # ~0.57 degrees total error
        print("✓ Angle-motor conversion PASSED")
    elif len(clamped) == 0:
        print("✗ Angle-motor conversion FAILED (but no clamping detected)")
    else:
        print("✗ Angle-motor conversion FAILED (positions were clamped to motor limits)")

    print("\n--- Testing Workspace Reachability ---")
    test_positions = [
        (np.array([100, 100, 150]), "Should be reachable"),
        (np.array([150, 0, 100]), "Should be reachable"),
        (np.array([500, 500, 500]), "Too far - unreachable"),
        (np.array([10, 10, 10]), "Too close - unreachable"),
    ]
    
    for position, description in test_positions:
        reachable = robot.is_position_reachable(position)
        status = "✓ Reachable" if reachable else "✗ Unreachable"
        print(f"{position}: {status} - {description}")
    
    print("\n" + "="*60)
    print("Software tests complete! No hardware required for above tests.")
    print("="*60)


def test_with_hardware():
    """Test with actual robot hardware"""
    print("\n" + "="*60)
    print("TEST 2: Hardware Connection Test")
    print("="*60)
    
    # IMPORTANT: Set your device name here
    device_name = "/dev/ttyUSB0"  # Change this for your system!
    # Windows: "COM7"
    # Linux: "/dev/ttyUSB0"
    # Mac: "/dev/tty.usbserial-*"
    
    print(f"\nAttempting to connect to: {device_name}")
    print("Make sure the robot is powered on and connected!")
    
    try:
        robot = RobotKinematics(device_name=device_name, baudrate=1_000_000)
        print("✓ RobotKinematics object created")
        
        print("\nInitializing robot...")
        robot.initialize()
        print("✓ Robot initialized")
        
        print("\nReading current position...")
        position, orientation = robot.get_current_pose()
        print(f"Current position: {position}")
        print(f"Current orientation:\n{orientation}")
        robot.print_current_state()
        print("✓ Position reading successful")
        
        print("\nTesting small movement...")
        # Move to a safe, known position
        safe_position = np.array([100, 100, 150])
        print(f"Moving to {safe_position}...")
        robot.move_to_position(safe_position, speed=100)
        print("✓ Movement successful")
        
        print("\nReading new position...")
        robot.print_current_state()
        
        print("\nReturning to home position...")
        robot.home_position()
        print("✓ Returned to home")
        
        print("\nClosing connection...")
        robot.close()
        print("✓ Connection closed")
        
        print("\n" + "="*60)
        print("✓ ALL HARDWARE TESTS PASSED!")
        print("="*60)
        
    except FileNotFoundError:
        print(f"\n✗ ERROR: Could not find device: {device_name}")
        print("Please check:")
        print("  1. Robot is powered on")
        print("  2. USB cable is connected")
        print("  3. Device name is correct for your system")
        print("  4. You have permissions to access the port (Linux: add user to dialout group)")
        
    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")
        print("\nTroubleshooting:")
        print("  - Check robot power")
        print("  - Verify correct device name")
        print("  - Check baudrate (should be 1000000)")
        print("  - Ensure no other program is using the port")


def main():
    print("\n" + "="*60)
    print("RobotKinematics Test Suite")
    print("="*60)
    
    print("\nThis script will test the RobotKinematics class.")
    print("\nTest 1: Software only (no hardware needed)")
    print("Test 2: Hardware connection test (requires robot)")
    
    # Always run software tests
    test_without_hardware()
    
    # Ask before running hardware tests
    print("\n" + "="*60)
    response = input("\nDo you want to run hardware tests? (y/n): ").strip().lower()
    
    if response == 'y':
        print("\nIMPORTANT: Before running hardware tests:")
        print("  1. Make sure robot is powered ON")
        print("  2. USB cable is connected")
        print("  3. No other program is using the port")
        print("  4. Robot has clear space to move")
        
        confirm = input("\nReady to proceed? (y/n): ").strip().lower()
        if confirm == 'y':
            test_with_hardware()
        else:
            print("Hardware tests cancelled.")
    else:
        print("\nHardware tests skipped.")
        print("\nTo run hardware tests later:")
        print("  1. Update device_name in test_with_hardware() function")
        print("  2. Run this script again and choose 'y'")
    
    print("\n" + "="*60)
    print("Testing complete!")
    print("="*60)


if __name__ == "__main__":
    main()
