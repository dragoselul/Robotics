import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from Dragos import util
import sympy as sp

if __name__ == "__main__":
    # Define symbolic variables
    m, L = sp.symbols('m L', real=True, positive=True)

    # Define inertia components using correct SymPy syntax
    ixx = m/12 * ((0.1*L)**2 + (0.1*L)**2)
    iyy = m/12 * ((0.1*L)**2 + L**2)
    izz = m/12 * ((0.1*L)**2 + L**2)

    # Create the inertia matrix
    inertia_matrix = sp.Matrix([[ixx, 0, 0], [0, iyy, 0], [0, 0, izz]])
    rotation_z = util.RotationMatrices.get_rotation_matrix_for_axis("z", sp.symbols("theta"))
    J1 = rotation_z * inertia_matrix * sp.transpose(rotation_z)
    # sp.pprint({sp.simplify(J1.evalf(3))}, use_unicode=True)
    L = sp.symbols("L")
    dh_table = [
        (sp.symbols("theta_1"), L, L, 0),  # a, alpha, d, theta
        (sp.symbols("theta_2"), L, L, 0)]

    dh_matrix = util.ForwardKinematics.forward_kinematics_from_dh(dh_table)

    # Debug: print the shape and structure of dh_matrix
    print("DH matrix shape:", dh_matrix.shape)
    print("DH matrix:")
    sp.pprint(dh_matrix)

    # Extract position vector correctly (4th column, first 3 rows)
    O1 = sp.Matrix([dh_matrix[0, 3], dh_matrix[1, 3], dh_matrix[2, 3]])
    print("\nPosition vector O1:")
    sp.pprint(O1)
