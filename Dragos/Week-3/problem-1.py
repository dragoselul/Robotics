import sys
import os

from Dragos.util import RotationMatrices

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from Dragos import util
import sympy as sp


def question_1():
    # Define symbols first so we can reference them later
    theta_1 = sp.symbols('theta_1')
    theta_2 = sp.symbols('theta_2')
    d_3 = sp.symbols('d_3')
    theta_4 = sp.symbols('theta_4')
    theta_5 = sp.symbols('theta_5')
    theta_6 = sp.symbols('theta_6')

    dh_table = [(theta_1, 0, 0, -sp.pi/2),
                (theta_2, 0.1, 0, sp.pi/2),
                (0, d_3, 0, 0),
                (theta_4, 0, 0, -sp.pi/2),
                (theta_5, 0, 0, sp.pi/2),
                (theta_6, 0, 0, 0)]

    return dh_table

def question_2(dh_table):
    T0_6 = util.ForwardKinematics.forward_kinematics_from_dh(dh_table)
    return T0_6


"""
The rotation matrix of the end-effector with respect to the initial frame can be
written as a function of the Yaw-Pitch-Roll angles ψ, θ, φ
"""
def question_3(fw_kinematics):
    R0_6 = RotationMatrices.get_rotation_matrix_for_axis("Z", sp.symbols("phi")) * RotationMatrices.get_rotation_matrix_for_axis("Y", sp.symbols("theta")) * RotationMatrices.get_rotation_matrix_for_axis("X", sp.symbols("psi"))
    return R0_6

if __name__ == "__main__":
    dh_table = question_1()
    fw_kinematic = question_2(dh_table)
    R0_6 = question_3(fw_kinematic)
    sp.print_latex(R0_6)