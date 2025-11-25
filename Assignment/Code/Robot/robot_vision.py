import numpy as np

class RobotVision:
    def __init__(self, robot_formulas):
        """
        Handles Vision <-> Kinematics math.
        Args:
            robot_formulas: Instance of RobotFormulas class (for FK)
        """
        self.kin = robot_formulas

        # Camera Intrinsic Matrix (K)
        # Placeholder values - UPDATED to be more realistic for 640x480
        # If uncalibrated, cx/cy should be exactly half width/height
        self.fx = 770.0 
        self.fy = 770.0
        self.cx = 320.0
        self.cy = 240.0  

        # Transformation from End-Effector (Frame 4) to Camera Frame (Frame 5)
        # --------------------------------------------------------------------
        # IMPORTANT: Define how the camera is rotated relative to the End Effector.
        # 
        # Case A: Camera looks exactly same direction as tool tip (Z_cam == Z_tool)
        # R_cam_tool = Identity (No rotation)
        #
        # Case B: Camera is mounted on top, but Frame 4 Z points forward and Camera looks down?
        # You need to check your FK T04 definition. 
        # Usually T04 Z points along the last link.
        #
        # Let's assume Identity for now (Camera looks along End Effector Z axis).
        # If your camera is rotated 90deg, change this matrix.
        self.R_cam_tool = np.eye(3) 
        
        # Offset: Camera is often shifted relative to tool tip (e.g., 4.5cm up/back)
        # Adjust this based on physical mounting! 
        # Example: 4.5cm back along X axis of tool
        self.p_cam_tool = np.array([0.045, 0, 0]) 

    def set_calibration(self, fx, fy, cx, cy):
        """Update camera intrinsics after calibration"""
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def pixel_to_ray_camera_frame(self, u, v):
        """
        Converts pixel (u,v) to a normalized 3D vector in Camera Frame.
        """
        x_c = (u - self.cx) / self.fx
        y_c = (v - self.cy) / self.fy
        z_c = 1.0
        return np.array([x_c, y_c, z_c])

    def get_key_position(self, u, v, current_joint_angles, keyboard_z_height=0.0):
        """
        Calculates the real-world (X, Y) position of a key seen at pixel (u, v).
        """
        # 1. Get End-Effector Pose (T04)
        # T04 is 4x4 Matrix: [Rotation (3x3) | Position (3x1)]
        T04, _ = self.kin.forward_kinematics(current_joint_angles)

        # Extract Tool Rotation and Position
        R_tool_base = T04[:3, :3]
        p_tool_base = T04[:3, 3]

        # 2. Calculate Camera Pose in Base Frame (T05 equivalent)
        # R_cam_base = R_tool_base * R_cam_tool
        R_cam_base = R_tool_base @ self.R_cam_tool
        
        # p_cam_base = p_tool_base + R_tool_base * p_cam_tool_offset
        p_cam_base = p_tool_base + (R_tool_base @ self.p_cam_tool)

        # DEBUG: Check Camera Looking Direction (Z-axis of Camera Frame)
        cam_z_world = R_cam_base[:, 2] 
        print(f"DEBUG: Camera Origin: {p_cam_base}")
        print(f"DEBUG: Camera Looking Vec (Z): {cam_z_world}")

        # SAFETY CHECK: Is camera actually looking down?
        # If Z component is positive (looking up) or near 0 (horizon), abort.
        # We expect a value like -0.9 (looking mostly down).
        if cam_z_world[2] > -0.2:
            print(f"⚠ ERROR: Camera is looking UP or Horizontal (Z={cam_z_world[2]:.2f}).")
            print("  -> Cannot calculate intersection with table.")
            print("  -> Move robot to a top-down scanning pose first.")
            return None

        # 3. Get Ray Vector in Camera Frame
        ray_cam = self.pixel_to_ray_camera_frame(u, v)

        # 4. Transform Ray to Base Frame
        ray_base = R_cam_base @ ray_cam

        # Normalize ray direction
        ray_dir = ray_base / np.linalg.norm(ray_base)

        # 5. Ray-Plane Intersection
        # We want t such that: p_cam_z + t * ray_dir_z = keyboard_z
        
        # Check for division by zero (ray parallel to plane)
        if abs(ray_dir[2]) < 1e-6:
            print("⚠ Error: Ray is parallel to table plane.")
            return None

        t = (keyboard_z_height - p_cam_base[2]) / ray_dir[2]

        if t < 0:
            print("⚠ Error: Intersection is BEHIND the camera.")
            return None

        # 6. Calculate Hit Point
        key_position = p_cam_base + t * ray_dir

        print(f"DEBUG: Intersection found at t={t:.3f}m -> {key_position}")

        return key_position
