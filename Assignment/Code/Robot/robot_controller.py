"""
Robot Kinematics Controller
High-level controller that bridges mathematical formulas with your Robot hardware class.
"""

import numpy as np
import time

# Support both package and direct script execution
try:
    from .kinematics import RobotKinematics
    from .translation import rad_to_motor_units, motor_units_to_rad
    from .robot_vision import RobotVision
except ImportError:  # fallback when run as a standalone script
    from kinematics import RobotKinematics
    from translation import rad_to_motor_units, motor_units_to_rad
    from robot_vision import RobotVision


class RobotKinematicsController:
    """
    High-level controller.
    - Uses RobotFormulas for math (IK, FK, Trajectory).
    - Uses Robot class for hardware communication.
    """

    def __init__(self, robot, verbose=False, mock=True):
        """
        Args:
            robot: Instance of your Robot class
            verbose: Print debug info
            mock: If True, don't send commands to hardware
        """
        self.robot = robot
        self.verbose = verbose
        self.mock = mock

        # Initialize Math Engine
        self.current_joint_angles = [2.61799, 2.61799, 2.61799, 1]  # [q1, q2, q3, q4] in radians
        self.home_angles = [2.61799, 2.61799, 2.61799, 1]  
        self.kin = RobotKinematics(home_angles=self.home_angles, verbose=verbose)
        self.vision = RobotVision(self.kin)

        # State tracking
        # Home position
        
        # World coordinate system (robot base is origin by default)
        self.world_origin = {
            'position': [0, 0, 0],  # x, y, z in meters
            'description': 'Base of robot, motor 1 rotation axis',
            'x_axis': 'Points forward from robot base',
            'y_axis': 'Points left (right-hand rule)',
            'z_axis': 'Points upward (vertical)'
        }
        
        # Robot base position in world coordinates (if different from world origin)
        self.robot_base_in_world = np.array([0.0, 0.0, 0.0])
        
        # Workspace calibration data
        self.calibration_points = {}
        self.workspace = {}

    # ========================================================================
    # INITIALIZATION & SHUTDOWN
    # ========================================================================

    def initialize(self):
        """Initialize robot, enable torque, and go to home."""
        print("Initializing Robot Controller...")
        self.robot.initialize()  # Calls your Robot.initialize() (opens COM, sets speed)
        self.move_to_home()
        # Ensure torque is enabled for control
        self.robot.enable_torque([1, 2, 3, 4])

        # Read actual starting position
        self.update_current_state()
        print(f"Robot initialized at: {np.degrees(self.current_joint_angles)}")

    def close(self):
        """Disable torque and close connection."""
        print("Closing Robot Controller...")
        self.robot.disable_torque([1, 2, 3, 4])
        self.robot.close()

    # ========================================================================
    # COORDINATE SYSTEM & CALIBRATION
    # ========================================================================

    def calibrate_workspace(self):
        """
        Manually move robot to key positions and record them.
        Use this to build your coordinate system.
        
        Returns:
            Dictionary of calibration points
        """
        calibration_points = {}
        
        print("\n=== Workspace Calibration ===")
        print("Move robot to each position and press Enter:")
        
        # Home position\
        self.robot.disable_torque([1, 2, 3, 4])
        input("\n1. Move to HOME/ORIGIN position, then press Enter...")
        calibration_points['origin'] = self.get_end_effector_pose().tolist()
        print(f"   Recorded: {calibration_points['origin']}")
        self.robot.enable_torque([1, 2, 3, 4])
        
        # Workspace corners
        self.robot.disable_torque([1, 2, 3, 4])
        input("\n2. Move to FRONT-LEFT corner of keyboard, press Enter...")
        calibration_points['keyboard_fl'] = self.get_end_effector_pose().tolist()
        print(f"   Recorded: {calibration_points['keyboard_fl']}")
        self.robot.enable_torque([1, 2, 3, 4])
        
        self.robot.disable_torque([1, 2, 3, 4])
        input("\n3. Move to FRONT-RIGHT corner of keyboard, press Enter...")
        calibration_points['keyboard_fr'] = self.get_end_effector_pose().tolist()
        print(f"   Recorded: {calibration_points['keyboard_fr']}")
        self.robot.enable_torque([1, 2, 3, 4])
        
        self.robot.disable_torque([1, 2, 3, 4])
        input("\n4. Move to BACK-LEFT corner of keyboard, press Enter...")
        calibration_points['keyboard_bl'] = self.get_end_effector_pose().tolist()
        print(f"   Recorded: {calibration_points['keyboard_bl']}")
        self.robot.enable_torque([1, 2, 3, 4])
        
        self.robot.disable_torque([1, 2, 3, 4])
        input("\n5. Move to BACK-RIGHT corner of keyboard, press Enter...")
        calibration_points['keyboard_br'] = self.get_end_effector_pose().tolist()
        print(f"   Recorded: {calibration_points['keyboard_br']}")
        self.robot.enable_torque([1, 2, 3, 4])
        # Calculate workspace dimensions
        self._calculate_workspace_from_calibration(calibration_points)

        
        # Save to file
        np.save('calibration.npy', calibration_points)
        print(f"\n✓ Calibration saved to 'calibration.npy'")
        print(f"  Workspace: {self.workspace}")
        
        self.calibration_points = calibration_points
        return calibration_points

    def load_calibration(self, filename='calibration.npy'):
        """Load previously saved calibration data."""
        try:
            self.calibration_points = np.load(filename, allow_pickle=True).item()
            self._calculate_workspace_from_calibration(self.calibration_points)
            print(f"✓ Calibration loaded from '{filename}'")
            print(f"  Workspace: {self.workspace}")
            return self.calibration_points
        except FileNotFoundError:
            print(f"⚠ Calibration file '{filename}' not found. Run calibrate_workspace() first.")
            return None

    def _calculate_workspace_from_calibration(self, cal_points):
        """Calculate workspace bounds from calibration points."""
        if len(cal_points) < 4:
            return
        
        # Extract positions
        points = np.array([
            cal_points.get('keyboard_fl', [0,0,0]),
            cal_points.get('keyboard_fr', [0,0,0]),
            cal_points.get('keyboard_bl', [0,0,0]),
            cal_points.get('keyboard_br', [0,0,0])
        ])
        
        # Calculate center and dimensions
        self.workspace = {
            'center': np.mean(points, axis=0).tolist(),
            'x_min': np.min(points[:, 0]),
            'x_max': np.max(points[:, 0]),
            'y_min': np.min(points[:, 1]),
            'y_max': np.max(points[:, 2]),
            'z_height': np.mean(points[:, 2]),
            'x_range': np.max(points[:, 0]) - np.min(points[:, 0]),
            'y_range': np.max(points[:, 1]) - np.min(points[:, 1])
        }

    # ========================================================================
    # MOVEMENT METHODS (The "Business Logic")
    # ========================================================================

    def move_joints(self, joint_angles):
        """
        Move robot to specified joint angles (radians).
        Handles conversion and calls robot.move()
        """
        # 1. Convert Radians -> Motor Units
        motor_positions = self.angles_to_motor_positions(joint_angles)
        if self.verbose:
            print(f"Moving to motor positions: {motor_positions}")
        self.robot.set_move_speed([1, 2, 3, 4], speed=150)  # Set a reasonable speed
        
        # 2. Send to Hardware
        if not self.mock:
            self.robot.move(motor_positions)

        # 3. Update Internal State
        self.current_joint_angles = joint_angles

        if self.verbose:
            print(f"Moving to angles (deg): {np.degrees(joint_angles)}")

    def move_to_home(self):
        """Go to home position (2.61799, 2.61799, 2.61799, 2.61799 rad)."""
        print("Moving to Home...")
        self.move_joints(self.home_angles)

    def move_to_position(self, x, y, z, elbow_up=True):
        """
        Move to position in ROBOT BASE coordinates.
        
        Args:
            x, y, z: Target position in robot base frame (meters)
            elbow_up: Elbow configuration
        """
        target = [x, y, z]

        # 1. Calculate IK
        q = self.kin.inverse_kinematics(target)
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

    def move_to_world_position(self, world_x, world_y, world_z, elbow_up=True):
        """
        Move to a position specified in WORLD coordinates.
        Automatically transforms to robot base frame.
        
        Args:
            world_x, world_y, world_z: Target in world frame (meters)
            elbow_up: Elbow configuration
        """
        # Transform world coords -> robot coords
        robot_coords = self.transform_world_to_robot([world_x, world_y, world_z])
        
        if self.verbose:
            print(f"World: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f}) -> Robot: ({robot_coords[0]:.3f}, {robot_coords[1]:.3f}, {robot_coords[2]:.3f})")
        
        # Use existing method
        return self.move_to_position(robot_coords[0], robot_coords[1], robot_coords[2], elbow_up)

    def move_to_position_smooth(self, x, y, z, duration=3.0, use_world_coords=False):
        """
        Smoothly interpolate from CURRENT position to TARGET position.
        Uses Quintic Polynomial trajectory.
        
        Args:
            x, y, z: Target position (meters)
            duration: Movement time (seconds)
            use_world_coords: If True, x/y/z are in world frame; if False, robot base frame
        """
        
        target = [x, y, z]
        q_target = self.kin.inverse_kinematics(target, orientation=)

        if q_target is None:
            print(f"⚠ Target {target} unreachable!")
            return False

        # Generate Trajectory from current state
        print(f"Executing Smooth Trajectory to {target} over {duration}s...")
        self.execute_trajectory(self.current_joint_angles, q_target, duration)
        return True

    def execute_trajectory(self, q_start, q_end, duration):
        """
        Generates and executes a time-based trajectory.
        
        Args:
            q_start: Starting joint angles [theta1, theta2, theta3, theta4] (radians)
            q_end: Ending joint angles [theta1, theta2, theta3, theta4] (radians)
            duration: Total trajectory time (seconds)
        """
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
        self.move_joints(q_end)
        print("✓ Trajectory Finished.")

    # ========================================================================
    # SENSORS & FEEDBACK
    # ========================================================================

    def update_current_state(self):
        """Reads actual motor positions and updates current_joint_angles."""
        if self.mock:
            return self.current_joint_angles
        
        # 1. Read from hardware
        positions_dict = self.robot.read_current_position([1, 2, 3, 4])

        # 2. Convert to radians
        self.current_joint_angles = self.motor_units_to_angles(positions_dict)
        return self.current_joint_angles

    # ========================================================================
    # HELPERS: CONVERSION
    # ========================================================================

    def angles_to_motor_positions(self, joint_angles):
        """Convert [q1,q2,q3,q4] array to {1:pos, 2:pos...} dict."""
        motor_positions = {}
        for i, angle in enumerate(joint_angles):
            motor_id = i + 1
            val = rad_to_motor_units(angle)
            motor_positions[motor_id] = val
        return motor_positions

    def motor_units_to_angles(self, positions_dict):
        """Convert {1:pos, 2:pos...} dict to [q1,q2,q3,q4] array."""
        angles = np.zeros(4)
        for motor_id, pos in positions_dict.items():
            idx = motor_id - 1
            if 0 <= idx < 4:
                angles[idx] = motor_units_to_rad(pos)
        return angles

    # ========================================================================
    # HIGH-LEVEL TASKS
    # ========================================================================

    def press_key_at_pixel(self, u, v, keyboard_height=0.0):
        """
        High-level command: Sees a key at (u,v), calculates position, and presses it.
        
        Args:
            u, v: Pixel coordinates from camera
            keyboard_height: Height of keyboard surface (z coordinate)
        """
        # 1. Calculate real world position of the key
        key_pos = self.vision.get_key_position(
            u, v,
            self.current_joint_angles,
            keyboard_height
        )

        if key_pos is None:
            print("⚠ Failed to calculate key position")
            return False

        print(f"Key detected at world pos: {key_pos}")

        # 2. Hover above key (e.g., 5cm up)
        hover_pos = [key_pos[0], key_pos[1], keyboard_height + 0.05]
        if not self.move_to_position_smooth(hover_pos[0], hover_pos[1], hover_pos[2]):
            return False

        # 3. Press down (e.g., 1cm deep)
        press_pos = [key_pos[0], key_pos[1], keyboard_height - 0.01]
        if not self.move_to_position_smooth(press_pos[0], press_pos[1], press_pos[2], duration=0.5):
            return False

        # 4. Return to hover
        self.move_to_position_smooth(hover_pos[0], hover_pos[1], hover_pos[2], duration=0.5)

        return True

    def move_to_pixel(self, u, v, keyboard_z_height=0.0):
        """
        Moves the robot end-effector to the physical location of a pixel (u, v).
        
        Args:
            u, v: Pixel coordinates from the camera image.
            keyboard_z_height: The physical height of the keyboard (z-axis).
                               Use your calibrated z_height (e.g., 0.122).
        """
        print(f"Vision: Calculating position for pixel ({u}, {v})...")

        # 1. Get current joint angles (needed for camera pose T05)
        current_q = self.update_current_state()

        # 2. Use RobotVision to calculate real-world [x, y, z]
        # Note: Ensure self.vision is initialized in __init__
        target_pos = self.vision.get_key_position(
            u, v, 
            current_q, 
            keyboard_z_height
        )

        if target_pos is None:
            print("Vision: Could not calculate target position.")
            return False

        print(f"Vision: Pixel ({u}, {v}) -> World {target_pos}")

        # 3. Add an offset for "Hover" vs "Press"
        # Move to a hover position first (e.g., 5cm above the key)
        hover_z = target_pos[2] + 0.05
        
        # Move to Hover
        success = self.move_to_position_smooth(
            target_pos[0], target_pos[1], hover_z, 
            duration=2.0
        )
        
        if not success:
            print("Vision: Failed to move to hover position.")
            return False

        # Optional: Press the key (move down to target_pos[2])
        # ... add press logic here if you want ...

        return True


    def execute_waypoint_sequence(self, waypoints, use_world_coords=False, duration_per_segment=2.0):
        """
        Execute a sequence of waypoints with smooth trajectories.
        
        Args:
            waypoints: List of [x, y, z] positions
            use_world_coords: If True, waypoints are in world frame
            duration_per_segment: Time for each segment (seconds)
        
        Returns:
            True if all waypoints reached successfully
        """
        print(f"\n=== Executing Waypoint Sequence ({len(waypoints)} points) ===")
        
        for i, waypoint in enumerate(waypoints):
            print(f"Waypoint {i+1}/{len(waypoints)}: {waypoint}")
            
            success = self.move_to_position_smooth(
                waypoint[0], waypoint[1], waypoint[2],
                duration=duration_per_segment,
                use_world_coords=use_world_coords
            )
            
            if not success:
                print(f"⚠ Failed to reach waypoint {i+1}")
                return False
            
            time.sleep(0.5)  # Brief pause at each waypoint
        
        print("✓ Waypoint sequence completed")
        return True
