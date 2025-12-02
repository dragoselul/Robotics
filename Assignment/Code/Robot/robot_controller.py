"""
Robot Kinematics Controller
High-level controller that bridges mathematical formulas with your Robot hardware class.
"""

import numpy as np
import time

try:
    from .kinematics import RobotKinematics
    from .translation import rad_to_motor_units, motor_units_to_rad
    from .robot_vision import RobotVision
except ImportError:
    from kinematics import RobotKinematics
    from translation import rad_to_motor_units, motor_units_to_rad
    from robot_vision import RobotVision


class RobotKinematicsController:

    def __init__(self, robot, verbose=False, mock=False):
        self.robot = robot
        self.verbose = verbose
        self.mock = mock

        # Initialize Math Engine
        self.current_joint_angles = [0,0,0,0]  # [q1, q2, q3, q4] in radians
        self.kin = RobotKinematics(verbose=verbose)
        self.vision = RobotVision(self.kin)

    def initialize(self):
        print("Initializing Robot Controller...")
        self.robot.initialize()
        self.move_to_position(140, 0, 125, [1, 0, 0])
        self.robot.enable_torque([1, 2, 3, 4])

        self.update_current_state()
        print(f"Robot initialized at: {np.degrees(self.current_joint_angles)}")

    def close(self):
        print("Closing Robot Controller...")
        self.robot.disable_torque([1, 2, 3, 4])
        self.robot.close()

    def move_joints(self, joint_angles, speed=150):
        motor_positions = self.angles_to_motor_positions(joint_angles)
        if self.verbose:
            print(f"Moving to motor positions: {motor_positions}")
        self.robot.set_move_speed([1, 2, 3, 4], speed=speed)  # Set a reasonable speed

        if not self.mock:
            self.robot.move(motor_positions)

        self.current_joint_angles = joint_angles

        if self.verbose:
            print(f"Moving to angles (deg): {np.degrees(joint_angles)}")

    def move_to_position(self, x, y, z, orientation=None):
        target = [x, y, z]

        # 1. Calculate IK
        q = self.kin.inverse_kinematics(target, orientation=orientation)
        if q is None:
            print(f"⚠ IK failed for target {target}")
            return False

        # 2. Verify with FK (Sanity Check)
        T04, _ = self.kin.forward_kinematics(q)
        computed_pos = T04[:3, 3]
        error = np.linalg.norm(np.array(target) - computed_pos)
        if error > 0.001:  # 1mm tolerance
            print(f"⚠ IK Error too high: {error*1000:.2f}mm")
            return False

        # 3. Check Singularity (Jacobian)
        J = self.kin.compute_jacobian(q)
        if np.linalg.cond(J) > 50:  # Threshold depends on your robot
            print("⚠ Configuration is singular!")
            return False

        # 4. Check Torques (Optional safety)
        tau = self.kin.compute_static_torques(q, force_vector=[0, 0, -5])
        if np.max(np.abs(tau)) > 1.5:  # 1.5Nm limit (example for AX-12)
            print(f"⚠ Torque overload predicted: {np.max(np.abs(tau)):.2f}Nm")
            return False

        # 5. Finally Move
        self.move_joints(q)
        return q

    def move_to_position_smooth(self, x, y, z, orientation=None, duration=3.0, speed=150):
        target = [x, y, z]
        q_target = self.kin.inverse_kinematics(target, orientation=orientation)

        if q_target is None:
            print(f"⚠ Target {target} unreachable!")
            return False

        print(f"Executing Smooth Trajectory to {target} over {duration}s...")
        self.execute_trajectory(self.current_joint_angles, q_target, duration, speed=speed)
        return True

    def execute_trajectory(self, q_start, q_end, duration, speed=150):
        dt = 0.05  # 20Hz control loop
        times, q_traj = self.kin.generate_quintic_trajectory(q_start, q_end, duration, dt)

        for q_step in q_traj:
            start_time = time.time()

            # Move to this step
            self.move_joints(q_step)

            # Wait to maintain timing
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

        # Ensure we hit the exact end point
        self.move_joints(q_end, speed=speed)
        print("✓ Trajectory Finished.")


    def update_current_state(self):
        if self.mock:
            return self.current_joint_angles
        
        # 1. Read from hardware
        positions_dict = self.robot.read_current_position([1, 2, 3, 4])

        # 2. Convert to radians
        self.current_joint_angles = self.motor_units_to_angles(positions_dict)
        return self.current_joint_angles

    def get_end_effector_pose(self):
        self.update_current_state()  # Get fresh data
        T04, _ = self.kin.forward_kinematics(self.current_joint_angles)
        print(f"End Effector Pose (Robot Frame): {T04[:3, 3]}")
        return T04[:3, 3]


    def angles_to_motor_positions(self, joint_angles):
        motor_positions = {}
        for i, angle in enumerate(joint_angles):
            motor_id = i + 1
            val = rad_to_motor_units(angle)
            motor_positions[motor_id] = 512 + val 
        return motor_positions

    def motor_units_to_angles(self, positions_dict):
        angles = np.zeros(4)
        for motor_id, pos in positions_dict.items():
            idx = motor_id - 1
            if 0 <= idx < 4:
                angles[idx] = motor_units_to_rad(pos - 512)
        return angles
