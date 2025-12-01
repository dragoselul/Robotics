"""
1. Forward Kinematics (End-Effector & Camera)
2. Inverse Kinematics (Geometric 3-DOF + Orientation)
3. Jacobian Calculation (Analytical)
4. Trajectory Planning (Quintic Polynomials)
5. Dynamics (Placeholder for torques)
"""

import numpy as np
import transfomations


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

        T01, T02, T03, T04, T05= transfomations.transformation(q1, q2, q3, q4)

        return T04, T05

    def get_end_effector_pos(self, q):
        T04, _ = self.forward_kinematics(q)
        return T04[:3, 3]

    # ========================================================================
    # 3. INVERSE KINEMATICS (Problem 2 & 3)
    # ========================================================================
    def inverse_kinematics(self, target_pos, orientation):
        """
        ALWAYS ELBOW UP - hardcoded inside the transformations script
        Standard geometric IK solution.
        Orientation: unit vector 1x3 that gives me the stylus pointing direction
        Self has information about the geomtry of the robot
        transformations script models all the kinematics of the robot
        """
        
        q = transfomations.IK4(target_pos, orientation)

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

# if __name__ == "__main__":
#     robot = RobotKinematics()

#     # 1. Define Task (e.g., Circle Point)
#     target = [0.150, 0.0, 0.120]  # x, y, z

#     # 2. Calculate IK
#     q = robot.inverse_kinematics(target)
#     print(f"Joint Angles: {np.degrees(q)}")

#     # 3. Verify with FK
#     T04, _ = robot.forward_kinematics(q)
#     print(f"FK Position: {T04[:3, 3]}")

#     # 4. Calculate Jacobian
#     J = robot.compute_jacobian(q)
#     print(f"Jacobian Condition: {np.linalg.cond(J)}")

#     # 5. Calculate Required Torques for 1N Load
#     tau = robot.compute_static_torques(q)
#     print(f"Static Torques: {tau}")
