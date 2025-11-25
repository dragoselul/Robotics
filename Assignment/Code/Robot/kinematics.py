"""
1. Forward Kinematics (End-Effector & Camera)
2. Inverse Kinematics (Geometric 3-DOF + Orientation)
3. Jacobian Calculation (Analytical)
4. Trajectory Planning (Quintic Polynomials)
5. Dynamics (Placeholder for torques)
"""

import numpy as np


class RobotKinematics:
    def __init__(self,home_angles, verbose=False):
        self.verbose = verbose
        self.home_angles = home_angles
        # ====================================================================
        # 1. ROBOT PARAMETERS (Figure 1)
        # ====================================================================
        self.d1 = 0.050  # Base height (meters)
        self.a2 = 0.093  # Shoulder length
        self.a3 = 0.093  # Elbow length
        self.a4 = 0.050  # Stylus length
        self.d_cam = 0.045  # Camera offset (Problem 1)

        # Joint Limits (Safe ranges in radians)
        self.joint_limits = [
            (-np.pi, np.pi),  # q1: Base
            (-2 * np.pi / 3, 2 * np.pi / 3),  # q2: Shoulder
            (-2 * np.pi / 3, 2 * np.pi / 3),  # q3: Elbow
            (-np.pi / 2, np.pi / 2)  # q4: Wrist
        ]

    # ========================================================================
    # 2. FORWARD KINEMATICS (Problem 1)
    # ========================================================================

    def forward_kinematics(self, q):
        """
        Calculates T04 (Stylus) and T05 (Camera)
        Returns: (T04, T05) as 4x4 matrices
        """
        q1, q2, q3, q4 = q

        # Precompute trig functions
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)
        c234 = np.cos(q2 + q3 + q4)
        s234 = np.sin(q2 + q3 + q4)

        # --- T04: Stylus Tip ---
        r4 = self.a2 * c2 + self.a3 * c23 + self.a4 * c234
        z4 = self.d1 + self.a2 * s2 + self.a3 * s23 + self.a4 * s234

        T04 = np.array([
            [c1 * c234, -c1 * s234, s1, c1 * r4],
            [s1 * c234, -s1 * s234, -c1, s1 * r4],
            [s234, c234, 0, z4],
            [0, 0, 0, 1]
        ])

        # --- T05: Camera (Offset by d_cam along local x-axis of T04) ---
        # Camera is physically mounted on link 4
        # T05 = T04 * T_offset
        # T_offset = Translation(x=d_cam)

        p_cam = T04 @ np.array([self.d_cam, 0, 0, 1])
        T05 = T04.copy()
        T05[:3, 3] = p_cam[:3]

        return T04, T05

    def get_end_effector_pos(self, q):
        T04, _ = self.forward_kinematics(q)
        return T04[:3, 3]

    # ========================================================================
    # 3. INVERSE KINEMATICS (Problem 2 & 3)
    # ========================================================================
    def inverse_kinematics1(self, target_pos, elbow_up=True):
        """
        Standard geometric IK solution.
        """
        x, y, z = target_pos

        # 1. Base Angle
        q1 = np.arctan2(y, x)

        # 2. Wrist Center Position (Standard positive radius)
        r_target = np.sqrt(x**2 + y**2)
        
        # IMPORTANT: Horizontal constraint -> wrist is BEHIND target
        r_wrist = r_target - self.a4
        z_wrist = z - self.d1

        # Debug
        # print(f"DEBUG: r_target={r_target:.4f}, r_wrist={r_wrist:.4f}, z_wrist={z_wrist:.4f}")

        # 3. 2-Link Planar IK
        numerator = (r_wrist**2 + z_wrist**2 - self.a2**2 - self.a3**2)
        denominator = (2 * self.a2 * self.a3)
        
        # Safety check for divide by zero
        if abs(denominator) < 1e-6:
            return None
            
        cos_q3 = numerator / denominator

        if abs(cos_q3) > 1.0:
            if self.verbose:
                print(f"⚠ Target unreachable: {target_pos} (Reach: {np.sqrt(r_wrist**2 + z_wrist**2):.3f} > Max: {self.a2+self.a3:.3f})")
            return None

        if elbow_up:
            q3 = np.arccos(cos_q3)
        else:
            q3 = -np.arccos(cos_q3)

        # Calculate q2
        k1 = self.a2 + self.a3 * np.cos(q3)
        k2 = self.a3 * np.sin(q3)
        q2 = np.arctan2(z_wrist, r_wrist) - np.arctan2(k2, k1)

        # 4. Orientation (Standard Horizontal Constraint: q2+q3+q4 = 0)
        q4 = -(q2 + q3)
        
        # OR Vertical Constraint (if you switched to that): 
        # q4 = -np.pi/2 - (q2 + q3)

        # 5. Wrap angles to be closest to Home (The correct fix for 150deg home)
        # Helper function
        def wrap_to_home(angle, home_angle):
            candidates = [angle, angle + 2*np.pi, angle - 2*np.pi]
            return min(candidates, key=lambda a: abs(a - home_angle))
        
        # Apply wrapping
        q1 = wrap_to_home(q1, self.home_angles[0])
        q2 = wrap_to_home(q2, self.home_angles[1])
        q3 = wrap_to_home(q3, self.home_angles[2])
        q4 = wrap_to_home(q4, self.home_angles[3])

        q = np.array([q1, q2, q3, q4])

        return q

    def inverse_kinematics(self, target_pos, elbow_up=True):
        """
        Geometric IK with BACKWARDS REACH support.
        Matches robots where Home is ~150 degrees (reaching 'backwards').
        """
        x, y, z = target_pos

        # --- FORCE BACKWARDS REACH (The Fix) ---
        # Standard IK assumes reaching forward (positive radius).
        # Your robot is at 150deg, meaning it reaches "backwards" (negative radius).
        
        # 1. Flip Base Angle 180 degrees
        q1 = np.arctan2(y, x) + np.pi 
        
        # 2. Use Negative Radius for calculations
        r_target = -np.sqrt(x**2 + y**2)

        # ---------------------------------------

        # 3. Wrist Center Position
        r_wrist = r_target - self.a4
        z_wrist = z - self.d1

        print(f"  DEBUG: r_target={r_target:.4f}, r_wrist={r_wrist:.4f}, z_wrist={z_wrist:.4f}")
        print(f"  DEBUG: Link lengths: a2={self.a2}, a3={self.a3}, a4={self.a4}, d1={self.d1}")

        # 4. 2-Link Planar IK (Shoulder & Elbow)
        # Note: r_wrist is negative, but squaring it in Law of Cosines makes it positive
        # 3. 2-Link Planar IK
        numerator = (r_wrist**2 + z_wrist**2 - self.a2**2 - self.a3**2)
        denominator = (2 * self.a2 * self.a3)
        cos_q3 = numerator / denominator
        print(f"  DEBUG: numerator={numerator:.6f}, denominator={denominator:.6f}")
        print(f"  DEBUG: cos_q3={cos_q3:.6f} (must be in [-1, 1])")
        if abs(cos_q3) > 1.0:
            if self.verbose:
                print(f"⚠ Target unreachable: {target_pos}")
            return None

        if elbow_up:
            q3 = np.arccos(cos_q3)
        else:
            q3 = -np.arccos(cos_q3)

        # 5. Calculate q2 
        # arctan2(z, r) handles the negative radius correctly here!
        k1 = self.a2 + self.a3 * np.cos(q3)
        k2 = self.a3 * np.sin(q3)
        q2 = np.arctan2(z_wrist, r_wrist) - np.arctan2(k2, k1)

        # 6. Orientation
        q4 = -np.pi/2 - (q2 + q3)

        # 7. Standard Wrapping (0 to 360)
        q = np.array([q1, q2, q3, q4])
        
        # Simple modulo to keep it clean [0, 2pi]
        q = np.mod(q, 2 * np.pi)

        if self.verbose:
            print(f"  Target: {target_pos}")
            print(f"  IK Angles (deg): {np.degrees(q)}")

        return q






    # ========================================================================
    # 4. JACOBIAN
    # ========================================================================

    def compute_jacobian(self, q):
        """
        Analytical Jacobian J(q) relating joint vels to end-effector vels
        v = J * q_dot
        """
        q1, q2, q3, q4 = q

        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)
        c234 = np.cos(q2 + q3 + q4)
        s234 = np.sin(q2 + q3 + q4)

        # Derivatives of position w.r.t q1..q4
        # x = c1 * (a2*c2 + a3*c23 + a4*c234)
        # y = s1 * (a2*c2 + a3*c23 + a4*c234)
        # z = d1 + a2*s2 + a3*s23 + a4*s234

        r = self.a2 * c2 + self.a3 * c23 + self.a4 * c234  # Radial distance

        # Partial derivatives
        dx_dq1 = -s1 * r
        dy_dq1 = c1 * r
        dz_dq1 = 0

        dr_dq2 = -(self.a2 * s2 + self.a3 * s23 + self.a4 * s234)
        dx_dq2 = c1 * dr_dq2
        dy_dq2 = s1 * dr_dq2
        dz_dq2 = self.a2 * c2 + self.a3 * c23 + self.a4 * c234

        dr_dq3 = -(self.a3 * s23 + self.a4 * s234)
        dx_dq3 = c1 * dr_dq3
        dy_dq3 = s1 * dr_dq3
        dz_dq3 = self.a3 * c23 + self.a4 * c234

        dr_dq4 = -(self.a4 * s234)
        dx_dq4 = c1 * dr_dq4
        dy_dq4 = s1 * dr_dq4
        dz_dq4 = self.a4 * c234

        J = np.array([
            [dx_dq1, dx_dq2, dx_dq3, dx_dq4],
            [dy_dq1, dy_dq2, dy_dq3, dy_dq4],
            [dz_dq1, dz_dq2, dz_dq3, dz_dq4],
            # Orientation rows (angular velocity) would go here
            [0, s1, s1, s1],  # omega_x
            [0, -c1, -c1, -c1],  # omega_y
            [1, 0, 0, 0]  # omega_z
        ])

        return J

    def compute_joint_velocities(self, q, v_target):
        """
        Problem 5: q_dot = J_pseudo_inverse * v_target
        v_target: [vx, vy, vz, wx, wy, wz]
        """
        J = self.compute_jacobian(q)
        J_pinv = np.linalg.pinv(J)
        q_dot = J_pinv @ v_target
        return q_dot

    # ========================================================================
    # 5. TRAJECTORY PLANNING
    # ========================================================================

    def compute_trajectory_coeffs(self, t0, tf, q0, qf, v0, vf, a0, af):
        """
        Computes coefficients for a single quintic polynomial segment:
        q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        """
        T = tf - t0
        # System of equations for Quintic Polynomial
        # Solved form:
        b = np.array([
            qf - q0 - v0 * T - 0.5 * a0 * T ** 2,
            vf - v0 - a0 * T,
            af - a0
        ])

        M = np.array([
            [T ** 3, T ** 4, T ** 5],
            [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
            [6 * T, 12 * T ** 2, 20 * T ** 3]
        ])

        # Solve for c3, c4, c5
        x = np.linalg.solve(M, b)

        coeffs = np.array([q0, v0, 0.5 * a0, x[0], x[1], x[2]])
        return coeffs

    def evaluate_polynomial(self, coeffs, t):
        """Evaluates q(t) for given coefficients"""
        c0, c1, c2, c3, c4, c5 = coeffs
        return c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5

    # ========================================================================
    # 6. DYNAMICS (Problem 9 & 10)
    # ========================================================================

    def compute_static_torques(self, q, force_vector=[0, 0, -1]):
        """
        Problem 9: J_transpose * Force
        Force: [Fx, Fy, Fz, Mx, My, Mz]
        """
        J = self.compute_jacobian(q)
        # 6D Wrench vector (Force + Moment)
        wrench = np.array(force_vector + [0, 0, 0])

        # tau = J.T * F
        tau = J.T @ wrench
        return tau

    def generate_quintic_trajectory(self, q_start, q_end, duration, dt=0.05):
        """
        Generates a full time-series trajectory for all 4 joints.

        Args:
            q_start: Start joint angles [q1, q2, q3, q4]
            q_end: End joint angles [q1, q2, q3, q4]
            duration: Total time (seconds)
            dt: Time step (seconds)

        Returns:
            times: Array of time stamps [0, dt, 2dt, ...]
            q_traj: Matrix of angles (N_steps x 4)
        """
        # 1. Create time steps
        times = np.arange(0, duration, dt)

        # 2. Initialize trajectory matrix (Rows=Time, Cols=Joints)
        q_traj = np.zeros((len(times), 4))

        # 3. Calculate trajectory for each joint independently
        for joint_idx in range(4):
            # Get start/end for this specific joint
            q0 = q_start[joint_idx]
            qf = q_end[joint_idx]

            # Compute coefficients (Assuming 0 velocity and 0 acceleration at boundaries)
            # v0=0, vf=0, a0=0, af=0
            coeffs = self.compute_trajectory_coeffs(
                t0=0, tf=duration,
                q0=q0, qf=qf,
                v0=0, vf=0,
                a0=0, af=0
            )

            # Evaluate polynomial for all time steps
            q_traj[:, joint_idx] = self.evaluate_polynomial(coeffs, times)

        return times, q_traj


# ============================================================================
# EXAMPLE USAGE (Template Method Pattern)
# ============================================================================

if __name__ == "__main__":
    robot = RobotKinematics()

    # 1. Define Task (e.g., Circle Point)
    target = [0.150, 0.0, 0.120]  # x, y, z

    # 2. Calculate IK
    q = robot.inverse_kinematics(target)
    print(f"Joint Angles: {np.degrees(q)}")

    # 3. Verify with FK
    T04, _ = robot.forward_kinematics(q)
    print(f"FK Position: {T04[:3, 3]}")

    # 4. Calculate Jacobian
    J = robot.compute_jacobian(q)
    print(f"Jacobian Condition: {np.linalg.cond(J)}")

    # 5. Calculate Required Torques for 1N Load
    tau = robot.compute_static_torques(q)
    print(f"Static Torques: {tau}")
