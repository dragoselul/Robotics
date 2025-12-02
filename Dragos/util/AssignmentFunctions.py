import numpy as np

"""
Numeric versions of assignment functions using NumPy.
For fast numeric calculations and robot control.
For symbolic mathematics, use AssignmentFunctionsSymbolic.py instead.
"""

"""
Skew-symmetric matrix
"""
def skew(x, y, z):
    return np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])


"""
Homogenous translation matrix
"""
def translation_matrix(x=0, y=0, z=0):
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


"""
Homogenous rotation matrix for x-axis
"""
def rotation_matrix_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ])


"""
Homogenous rotation matrix for y-axis
"""
def rotation_matrix_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ])


"""
Homogenous rotation matrix for z-axis
"""
def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


"""
Extracts the XYZ coordinates from the homogenous transformation matrix
A: Homogenous transformation matrix
Returns: XYZ coordinates
"""
def extract_XYZ(A):
    return A[0:3, 3]


"""
Extracts directional vectors from transformation matrix
"""
def extract_XYZ_vectors(A, vector_name):
    if vector_name == "x":
        return A[0:3, 0]
    elif vector_name == "y":
        return A[0:3, 1]
    elif vector_name == "z":
        return A[0:3, 2]
    else:
        raise ValueError("Invalid vector name")


"""
Cosine theorem solver - returns solutions for angle
"""
def cosTheorem(a, b, c):
    cos_theta = (b**2 + c**2 - a**2) / (2 * b * c)
    # Clamp to [-1, 1] to avoid numerical errors
    cos_theta = np.clip(cos_theta, -1, 1)
    return [np.arccos(cos_theta), -np.arccos(cos_theta)]


"""
DH transformation matrix using Denavit-Hartenberg parameters
"""
def DH(theta, d, a, alpha):
    cos_alpha, sin_alpha = np.cos(alpha), np.sin(alpha)
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    return np.array([
        [cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, a*cos_theta],
        [sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
        [0,         sin_alpha,             cos_alpha,            d],
        [0,         0,                     0,                    1]
    ])


"""
Forward kinematics - compute end-effector pose from joint angles
"""
def forward_kinematics(dh_matrices: list):
    T = np.eye(4)
    for matrix in dh_matrices:
        T = T @ matrix
    return T


"""
Hypotenuse calculation
"""
def hypot(x, y):
    return np.hypot(x, y)


"""
Inverse kinematics for joints 0->3
Takes a 3x1 array of XYZ coordinates and returns the inverse kinematics solutions
Returns: 2x3 array with two solutions (elbow down, elbow up)
"""
def inverse_kinematics0_3(origin_3):
    x, y, z = origin_3[0], origin_3[1], origin_3[2]
    theta_1 = np.arctan2(y, x)
    r = np.hypot(x, y)
    s = z - 50
    psi = np.arctan2(s, r)
    beta_list = cosTheorem(np.hypot(r, s), 93, 93)
    beta = beta_list[0]  # Take first solution
    phi = np.arctan2(93*np.sin(np.pi - beta), 93*np.cos(np.pi - beta) + 93)
    theta2_down = -np.pi / 2 + psi - phi
    theta2_up = -np.pi / 2 + psi + phi
    theta3_down = np.pi - beta
    theta3_up = -theta3_down
    return np.array([[theta_1, theta2_down, theta3_down],
                     [theta_1, theta2_up, theta3_up]])


"""
Inverse kinematics for 4-DOF robot
Compute inverse kinematics given desired end-effector pose
"""
def inverse_kinematics4(X_desired, origin_desired, T03_func=None):
    """
    Compute inverse kinematics for 4-DOF robot.

    Args:
        X_desired: desired x-axis vector of end-effector (3x1 numpy array) the direction of which is the desired orientation
        origin_desired: desired origin point of end-effector (3x1 numpy array)

    Returns:
        angles: numpy array [theta1, theta2, theta3, theta4]
    """
    L4 = 50  # link length along x4

    # Normalize X_des
    Xhat = X_desired / np.linalg.norm(X_desired)

    # 1) Wrist center
    origin_frame_3 = origin_desired - L4 * Xhat

    # 2) Solve IK03 for wrist center position
    sol03 = inverse_kinematics0_3(origin_frame_3)

    # Choose first branch (elbow up)
    t1, t2, t3 = sol03[0, 0], sol03[0, 1], sol03[0, 2]

    # 3) Compute T03 numerically using numpy
    if T03_func is not None:
        T03_numeric = T03_func(t1, t2, t3)
        T03_numeric_np = np.array(T03_numeric)
    else:
        T01 = DH(t1, 50, 0, np.pi/2)
        T12 = DH(t2, 0, 93, 0)
        T23 = DH(t3, 0, 93, 0)
        T03_numeric_np = T01 @ T12 @ T23

    R03 = T03_numeric_np[0:3, 0:3]  # Extract 3x3 rotation matrix

    # 4) Feasibility check: X4 must be perpendicular to z3
    z3 = R03[:, 2]
    if np.abs(np.dot(Xhat, z3)) > 1e-10:
        raise ValueError("Pose not achievable: desired x4 not perpendicular to z3 at wrist position.")

    # 5) Compute theta4 from projections on x3 and y3
    x3 = R03[:, 0]
    y3 = R03[:, 1]
    c4 = np.dot(Xhat, x3)
    s4 = np.dot(Xhat, y3)
    t4 = np.arctan2(s4, c4)

    angles = np.array([t1, t2, t3, t4])
    return angles


"""
Check if two values are approximately equal
"""
def aprox_equal(a, b, tolerance=1e-5):
    return abs(a - b) < tolerance


"""
Translation from point 3 to 4
"""
def translation_point34(vector, pos):
    k = 50 / np.linalg.norm(vector)
    point03 = pos - vector * k
    return point03


def Jvw(rotation_axis, end_effector, origin):
    """
    Compute the Jacobian column (linear and angular velocity) for a revolute joint.
    Arguments:
        rotation_axis: (3,) array representing joint axis (must be a unit vector)
        end_effector: (3,) array for end-effector position
        origin: (3,) array for this joint's frame origin
    Returns:
        jacobian: (6, 1) array, first 3 rows linear, last 3 angular
    """
    Jv = np.dot(skew(rotation_axis[0], rotation_axis[1], rotation_axis[2]), (end_effector - origin))
    Jw = rotation_axis
    jacobian = np.vstack((Jv.reshape(3, 1), Jw.reshape(3, 1)))
    return jacobian


def computeJacobian(theta1, theta2, theta3, theta4):
    # DH: function returning homogeneous transform (4x4 numpy array)
    # θ in radians
    T01 = DH(theta1, 50, 0, np.pi / 2)
    T12 = DH(theta2 + np.pi / 2, 0, 93, 0)
    T23 = DH(theta3, 0, 93, 0)
    T34 = DH(theta4, 0, 50, 0)
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)

    o0stylus = T04[:3, 3]
    o01 = T01[:3, 3]
    o02 = T02[:3, 3]
    o03 = T03[:3, 3]
    o00 = np.zeros(3)

    origins = [o00, o01, o02, o03]

    z00 = np.array([0, 0, 1])
    z01 = T01[:3, 2]
    z02 = T02[:3, 2]
    z03 = T03[:3, 2]
    axes = [z00, z01, z02, z03]

    full_jacobian = np.zeros((6, 4))

    for i in range(4):
        full_jacobian[:, i] = Jvw(axes[i], o0stylus, origins[i]).flatten()

    return full_jacobian

def numericJacobian(jacobian_func, pose):
    """
    Calculate the Jacobian numerically at a given pose using pure numpy functions.

    Parameters:
      jacobian_func: function that returns Jacobian as a (6,4) numpy array when evaluated at [theta1, theta2, theta3, theta4]
      pose: list or 1D array of joint values [theta1, theta2, theta3, theta4] (all in radians)

    Returns:
      numeric_jacob: (6,4) numpy array, rounded to 4 decimal places
    """
    jac_num = jacobian_func(*pose)  # pass the values directly
    jac_num = np.round(jac_num, 4) # mimic the rounding from MATLAB
    return jac_num

def configCalculator(points, T03):
    """
    Compute robot joint poses for a sequence of desired end-effector positions
    lying on the specified plane.

    Parameters:
      points: (3, N) numpy array with N points, each column is a point [x; y; z]
      T03:    Transformation/matrix/structure required by IK4_func
      IK4_func: function, signature (x_des, o_des, T03) → pose (list or np.array of joint angles)

    Returns:
      poses: (N, 4) numpy array where each row is [theta1, theta2, theta3, theta4]
    """
    poses = []
    N = points.shape[1]  # number of points

    for i in range(N):
        stylus_tip_vector = np.array([points[0, i], points[1, i], 0])
        pose = inverse_kinematics4(stylus_tip_vector, points[:, i], T03)  # shape (4,)
        poses.append(pose)

    return np.array(poses)  # (N, 4)
