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

        self.fx = 770.0 
        self.fy = 770.0
        self.cx = 320.0
        self.cy = 240.0  

        self.R_cam_tool = np.array([
            [ 0.0,  0.0,  1.0],  # Camera Z becomes Tool X (Depth = Reach)
            [-1.0,  0.0,  0.0],  # Camera X becomes Tool -Y
            [ 0.0, -1.0,  0.0]   # Camera Y becomes Tool -Z
        ])
        
        self.p_cam_tool = np.array([0.0, 0.045, 0.0])

    def get_target_from_stereo(self, joints_A, uv_A, joints_B, uv_B):
        """
        Calculates 3D target using triangulation from two different robot poses.
        """
        import cv2 # Ensure cv2 is imported!

        # --- Helper: Calculate Projection Matrix P = K * [R|t] ---
        def get_proj_matrix(joints):
            # 1. Base -> Tool (FK)
            T_base_tool, _ = self.kin.forward_kinematics(joints)
            
            # Unit Check (ensure Meters)
            if np.linalg.norm(T_base_tool[:3, 3]) > 10.0:
                 T_base_tool[:3, 3] /= 1000.0

            # 2. Tool -> Camera (Static Offset)
            T_tool_cam = np.eye(4)
            T_tool_cam[:3, :3] = self.R_cam_tool
            T_tool_cam[:3, 3]  = self.p_cam_tool
            
            # 3. Base -> Camera
            T_base_cam = T_base_tool @ T_tool_cam
            
            # 4. Camera -> Base (INVERSE)
            # OpenCV requires World-to-Camera (Extrinsics), not Camera-to-World
            T_cam_base = np.linalg.inv(T_base_cam)
            
            # 5. Build Projection Matrix (3x4)
            # P = K @ [R|t]
            K = np.array([[self.fx, 0, self.cx],
                          [0, self.fy, self.cy],
                          [0, 0, 1]])
            
            # [R|t] is the top 3 rows of the 4x4 inverse matrix
            R_t = T_cam_base[:3, :]
            
            return K @ R_t

        # --- Calculate Matrices for both views ---
        P1 = get_proj_matrix(joints_A)
        P2 = get_proj_matrix(joints_B)
        
        # --- Prepare Points for OpenCV ---
        pt1 = np.array(uv_A, dtype=float).reshape(2, 1)
        pt2 = np.array(uv_B, dtype=float).reshape(2, 1)
        
        # --- Triangulate ---
        # Returns 4D homogeneous coords (x, y, z, w)
        points_4d = cv2.triangulatePoints(P1, P2, pt1, pt2)
        
        # Convert to 3D (divide by w)
        X = points_4d[0] / points_4d[3]
        Y = points_4d[1] / points_4d[3]
        Z = points_4d[2] / points_4d[3]
        
        return np.array([X[0], Y[0], Z[0]])

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
    
    def get_object_pos_from_camera(self, u, v, radius_pixels, real_radius_meters, current_joints):
        """
        1. Calculate Object in Camera Frame (using radius for depth).
        2. Transform Object to Tool Frame (using fixed offset).
        3. Transform Object to Base Frame (using FK).
        Returns: [x, y, z] in MILLIMETERS
        """
        # --- 1. Vision Math (Camera Frame) ---
        # Calculate Depth (Z) in Meters based on known object size
        f_avg = (self.fx + self.fy) / 2.0
        z_cam = (f_avg * real_radius_meters) / radius_pixels
        
        # Reconstruct Point (X, Y, Z) in Camera Frame (Meters)
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy
        
        # P_cam: Vector from Camera Lens to Object
        p_cam = np.array([x_cam, y_cam, z_cam])

        # --- 2. Static Transform (Camera -> Tool) ---
        # Where is the object relative to the Tool Flange?
        # We apply the rotation (R_cam_tool) and offset (p_cam_tool)
        
        # Note: Ensure R_cam_tool is the "Look Forward" matrix we fixed earlier!
        p_obj_relative_to_tool = (self.R_cam_tool @ p_cam) + self.p_cam_tool
        
        # --- 3. Dynamic Transform (Tool -> Base) ---
        # Where is the Tool currently? (FK)
        T_base_tool, _ = self.kin.forward_kinematics(current_joints)
        
        # UNIT SAFETY: Detect if FK is in MM or Meters. We want METERS for math.
        if np.linalg.norm(T_base_tool[:3, 3]) > 10.0:
            T_base_tool[:3, 3] /= 1000.0
            
        R_base_tool = T_base_tool[:3, :3]
        p_tool_base = T_base_tool[:3, 3]
        
        # Calculate Absolute Object Position in Base Frame
        p_obj_base_meters = p_tool_base + (R_base_tool @ p_obj_relative_to_tool)
        
        # --- 4. Return in MILLIMETERS (For your Controller) ---
        return p_obj_base_meters * 1000.0

    def get_target_from_radius(self, u, v, radius_pixels, real_radius_meters, current_joint_angles):
        """
        Calculates 3D position of a target (e.g. circle center) given its known real size.
        Useful for objects not on the table (e.g. a hoop in the air).
        
        Args:
            u, v: Pixel coordinates of center
            radius_pixels: Detected radius in pixels
            real_radius_meters: Actual radius of the object in meters
            current_joint_angles: Current robot pose
            
        Returns:
            target_pos: [x, y, z] in Base Frame
        """
        # 1. Estimate Depth (Z in Camera Frame)
        # Z = (f * R_real) / R_pix
        # We average fx and fy for 'f'
        f_avg = (self.fx + self.fy) / 2.0
        z_cam = (f_avg * real_radius_meters) / radius_pixels
        
        # 2. Reconstruct Point in Camera Frame
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy
        
        p_cam = np.array([x_cam, y_cam, z_cam])
        
        # 3. Transform to Base Frame
        # Get Camera Pose
        T04, _ = self.kin.forward_kinematics(current_joint_angles)
        R_tool_base = T04[:3, :3]
        p_tool_base = T04[:3, 3]
        
        # Camera Pose in Base
        R_cam_base = R_tool_base @ self.R_cam_tool
        p_cam_base = p_tool_base + (R_tool_base @ self.p_cam_tool)
        
        # Transform Point
        p_base = p_cam_base + (R_cam_base @ p_cam)
        
        return p_base
