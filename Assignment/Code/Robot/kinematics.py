"""
1. Forward Kinematics (End-Effector & Camera)
2. Inverse Kinematics (Geometric 3-DOF + Orientation)
3. Jacobian Calculation (Analytical)
4. Trajectory Planning (Quintic Polynomials)
5. Dynamics (Placeholder for torques)
"""

import numpy as np


class RobotKinematics:
    def __init__(self, verbose=False):
        self.verbose = verbose
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

        dictionary_of_transformations= self.transformation(q1, q2, q3, q4)

        return dictionary_of_transformations["T04"], dictionary_of_transformations["T05"]

    def get_end_effector_pos(self, q):
        T04, _ = self.forward_kinematics(q)
        return T04[:3, 3]

    # ========================================================================
    # 3. INVERSE KINEMATICS (Problem 2 & 3)
    # ========================================================================

    def cos_theorem(self,  a, b, c):
        """
        Law of cosines:
        returns angle opposite side a in a triangle with sides a, b, c
        """
        try:
            if b == 0 and c == 0:
                return 0.0
            arccos = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))
        except Warning:
            print("Warning in cos_theorem with values:", a, b, c)
        return arccos

    def IK03(self, O, elbow_up=True):
        """
        Compute IK for frame 0→3 using trigonometry.
        O = [x, y, z] end-effector position in frame 0 (3×1 array-like)
        Returns a 2×3 numpy array:
            [theta1, theta2_up,   theta3_up  ]
            [theta1, theta2_down, theta3_down]
        """
        x, y, z = O

        # --- theta1 ---
        theta1 = np.arctan2(y, x)

        # --- compute r and s ---
        r = np.hypot(x, y)
        s = z - 50

        psi = np.arctan2(s, r)

        # using correct link lengths (93 and 93)
        beta = self.cos_theorem(np.hypot(r, s), 93, 93)

        # --- Compute phi ---
        phi = np.arctan2(93 * np.sin(np.pi - beta),
                         93 * np.cos(np.pi - beta) + 93)

        # --- theta2 up/down ---
        theta2_down = -np.pi / 2 + psi - phi
        theta2_up = -np.pi / 2 + psi + phi

        # --- theta3 up/down ---
        theta3_down = np.pi - beta
        theta3_up = -theta3_down

        if elbow_up is True:
            return np.array(
                [theta1, theta2_up, theta3_up])
        else:
            return np.array([theta1, theta2_down, theta3_down])

    # ===== TRANSFORMATIONS ====
    # not symbolic this time

    def translation_matrix_np(self, x, y, z):
        return np.array([
            [1., 0., 0., x],
            [0., 1., 0., y],
            [0., 0., 1., z],
            [0., 0., 0., 1.],
        ])

    def rotation_x_np(self,x):
        c, s = np.cos(x), np.sin(x)
        return np.array([
            [1., 0., 0., 0.],
            [0., c, -s, 0.],
            [0., s, c, 0.],
            [0., 0., 0., 1.],
        ])

    def rotation_y_np(self,y):
        c, s = np.cos(y), np.sin(y)
        return np.array([
            [c, 0., s, 0.],
            [0., 1., 0., 0.],
            [-s, 0., c, 0.],
            [0., 0., 0., 1.],
        ])

    def rotation_z_np(self, z):
        c, s = np.cos(z), np.sin(z)
        return np.array([
            [c, -s, 0., 0.],
            [s, c, 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.],
        ])

    def DH_np(self, theta, dz, dx, alpha):
        return (
                self.rotation_z_np(theta)
                @ self.translation_matrix_np(0., 0., dz)
                @ self.translation_matrix_np(dx, 0., 0.)
                @ self.rotation_x_np(alpha)
        )

    def transformation(self, theta1, theta2, theta3, theta4):
        """
        Compute all homogeneous transforms from base (0) to frame 4 (stylus tip).

        Returns a dict with:
          - "T01"   : 0 -> 1
          - "T02"   : 0 -> 2
          - "T03"   : 0 -> 3
          - "T04"   : 0 -> 4 (end of stylus)
        """

        # Relative DH transforms (same as MATLAB)
        T01 = self.DH_np(theta1, 50.0, 0.0, np.pi / 2)
        T12 = self.DH_np(theta2 + np.pi / 2, 0.0, 93.0, 0.0)
        T23 = self.DH_np(theta3, 0.0, 93.0, 0.0)
        T34 = self.DH_np(theta4, 0.0, 50.0, 0.0)

        # Absolute transforms wrt original base
        T02 = T01 @ T12
        T03 = T02 @ T23
        T04 = T03 @ T34  # <-- end of stylus
        T05 = T03 @ self.translation_matrix_np(35, 45, 0)

        return {
            "T01": T01,
            "T02": T02,
            "T03": T03,
            "T04": T04,
            "T05": T05
        }

    def inverse_kinematics(self, X_des, O_des, elbow_up=True):
        """
        X_des (ORIENTATION) : desired end-effector x-axis (3,) in frame {0}
        O_des (POSITION) : desired end-effector origin o_4 (3,) in frame {0}
        elbow_up : choose IK03 branch (True = elbow up, False = elbow down)
        base_offset_deg : base rotation offset (same as in transformations)

        Returns: [theta1, theta2, theta3, theta4]
        """

        L4 = 50.0  # link-4 length along x_4

        # Ensure numpy arrays of correct shape
        X_des = np.asarray(X_des, dtype=float).reshape(3)
        O_des = np.asarray(O_des, dtype=float).reshape(3)

        # 1) Normalize desired x_4
        Xhat = X_des / np.linalg.norm(X_des)

        # 2) Wrist center o3 = O_des - L4 * Xhat
        o3 = O_des - L4 * Xhat

        # 3) Solve 2R IK for {0} -> o3
        t1, t2, t3 = self.IK03(o3, elbow_up=elbow_up)

        # 4) Build numeric R03 for that branch using our FK
        # theta4 can be 0.0 here because T03 does not depend on theta4
        Ts = self.transformation(t1, t2, t3, 0.0)
        T03 = Ts["T03"]
        R03 = T03[:3, :3]

        # 5) Feasibility: x4 must be ⟂ z3
        z3 = R03[:, 2]
        if abs(np.dot(Xhat, z3)) > 1e-10:
            raise ValueError(
                "Pose not achievable: desired x4 not perpendicular to z3 at that wrist position."
            )

        # 6) θ4 from projections on x3, y3
        x3 = R03[:, 0]
        y3 = R03[:, 1]
        c4 = np.dot(Xhat, x3)
        s4 = np.dot(Xhat, y3)
        t4 = np.arctan2(s4, c4)

        return np.array([t1, t2, t3, t4])

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