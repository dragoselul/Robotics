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


    def set_calibration(self, fx, fy, cx, cy):
        """Update camera intrinsics after calibration"""
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
    
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
        p_obj_relative_to_tool = (self.R_cam_tool @ p_cam) + self.p_cam_tool
        
        # --- 3. Dynamic Transform (Tool -> Base) ---
        # Where is the Tool currently? (FK)
        T_base_tool, _ = self.kin.forward_kinematics(current_joints)

        if np.linalg.norm(T_base_tool[:3, 3]) > 10.0:
            T_base_tool[:3, 3] /= 1000.0
            
        R_base_tool = T_base_tool[:3, :3]
        p_tool_base = T_base_tool[:3, 3]
        
        p_obj_base_meters = p_tool_base + (R_base_tool @ p_obj_relative_to_tool)
        
        return p_obj_base_meters * 1000.0