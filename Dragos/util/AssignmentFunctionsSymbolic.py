import numpy as np
import sympy as sp

"""
Symbolic versions of assignment functions using SymPy.
Use these for symbolic mathematics, derivations, and analysis.
For numeric calculations and robot control, use AssignmentFunctions.py instead.
"""

"""
Skew-symmetric matrix - sympy version for symbolic calculations
"""
def skew(x, y, z):
    return sp.Matrix([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])


"""
Homogenous translation matrix - sympy version for symbolic calculations
"""
def translation_matrix(x=0, y=0, z=0):
    T = sp.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


"""
Homogenous rotation matrix for x-axis - sympy version
"""
def rotation_matrix_x(angle):
    R = sp.eye(4)
    R[1, 1] = sp.cos(angle)
    R[1, 2] = -sp.sin(angle)
    R[2, 1] = sp.sin(angle)
    R[2, 2] = sp.cos(angle)
    return R


"""
Homogenous rotation matrix for y-axis - sympy version
"""
def rotation_matrix_y(angle):
    R = sp.eye(4)
    R[0, 0] = sp.cos(angle)
    R[0, 2] = sp.sin(angle)
    R[2, 0] = -sp.sin(angle)
    R[2, 2] = sp.cos(angle)
    return R


"""
Homogenous rotation matrix for z-axis - sympy version
"""
def rotation_matrix_z(angle):
    R = sp.eye(4)
    R[0, 0] = sp.cos(angle)
    R[0, 1] = -sp.sin(angle)
    R[1, 0] = sp.sin(angle)
    R[1, 1] = sp.cos(angle)
    return R


"""
Extracts the XYZ coordinates from the homogenous transformation matrix - sympy version
"""
def extract_XYZ(A):
    return A[0:3, 3]


"""
Extracts directional vectors from transformation matrix - sympy version
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
Cosine theorem solver - symbolic version
"""
def cosTheorem(a, b, c):
    theta = sp.symbols('theta')
    eq = sp.Eq(a ** 2, b ** 2 + c ** 2 - 2 * b * c * sp.cos(theta))
    return sp.solve(eq, theta)


"""
DH transformation matrix - sympy version for symbolic calculations
"""
def DH(theta = sp.symbols('theta'), d = sp.symbols('d'), a = sp.symbols('a'), alpha = sp.symbols('alpha')) -> sp.Matrix:
    cos_alpha, sin_alpha = sp.cos(alpha), sp.sin(alpha)
    cos_theta, sin_theta = sp.cos(theta), sp.sin(theta)
    return sp.Matrix([
        [cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, a*cos_theta],
        [sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
        [0,         sin_alpha,             cos_alpha,            d],
        [0,         0,                     0,                    1]
    ])


"""
Forward kinematics - sympy version for symbolic calculations
"""
def forward_kinematics(dh_matrices: list):
    T = sp.eye(4)
    for matrice in dh_matrices:
        T = T * matrice
    # Only simplify at the end, not in the loop
    return sp.simplify(T)


"""
Hypotenuse calculation - symbolic version
"""
def hypot(x, y):
    return sp.sqrt(x**2 + y**2)


"""
Inverse kinematics 0->3 - sympy version for symbolic calculations
"""
def inverse_kinematics0_3(origin_3):
    x = origin_3[0]
    y = origin_3[1]
    z = origin_3[2]
    theta_1 = sp.atan2(y, x)
    r = hypot(x, y)
    s = z - 50
    psi = sp.atan(s/r)
    beta = cosTheorem(hypot(r, s), 93, 93)[0]  # Take first solution
    phi = sp.atan2(93*sp.sin(sp.pi - beta), 93*sp.cos(sp.pi - beta) + 93)
    theta2_down = -sp.pi / 2 + psi - phi
    theta2_up = -sp.pi / 2 + psi + phi
    theta3_down = sp.pi - beta
    theta3_up = -theta3_down
    return sp.Matrix([[theta_1, theta2_down, theta3_down],
                      [theta_1, theta2_up, theta3_up]])


"""
Symbolic approximate equality check
"""
def aprox_equal(a, b, tolerance=1e-5):
    return sp.Abs(a - b) < tolerance


"""
Translation point 3->4 - sympy version for symbolic calculations
"""
def translation_point34(vector, pos):
    vector_x = vector[0]
    vector_y = vector[1]
    vector_z = vector[2]

    k = 50 / sp.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
    point03 = pos - vector * k
    return point03


"""
Check transformation with 3 angles - substitute and evaluate
"""
def check3(T, theta_1, theta_2, theta_3):
    """Substitute numerical values into symbolic transformation matrix T and evaluate"""
    theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')

    # Substitute and evaluate in one step
    transformation = T.subs({theta1: theta_1, theta2: theta_2, theta3: theta_3}).evalf(4)

    return transformation


"""
Check transformation with 4 angles - substitute and evaluate
"""
def check4(T, theta_1, theta_2, theta_3, theta_4):
    """Substitute numerical values into symbolic transformation matrix T and evaluate"""
    theta1, theta2, theta3, theta4 = sp.symbols('theta1 theta2 theta3 theta4')

    # Substitute and evaluate in one step
    transformation = T.subs({
        theta1: theta_1,
        theta2: theta_2,
        theta3: theta_3,
        theta4: theta_4
    }).evalf(4)

    return transformation

