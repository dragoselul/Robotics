import sympy as sp


# A_i
class DenavitHartenbergTransformMatrix:
    def __init__(self):
        pass

        # ----- Classic DH single-step transform -----
    @staticmethod
    def get_A_dh(theta = sp.symbols('theta'), d = sp.symbols('d'), a = sp.symbols('a'), alpha = sp.symbols('alpha')):
        cos_alpha, sin_alpha = sp.cos(alpha), sp.sin(alpha)
        cos_theta, sin_theta = sp.cos(theta), sp.sin(theta)
        return sp.Matrix([
            [cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, a*cos_theta],
            [sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
            [0,         sin_alpha,         cos_alpha,        d],
            [ 0,      0,      0,    1]
        ])

class ForwardKinematics:
    def __init__(self):
        pass

    @staticmethod
    def forward_kinematics_from_dh(dh_table):
        T = sp.eye(4)
        for (a, alpha, d, theta) in dh_table:
            A = DenavitHartenbergTransformMatrix.get_A_dh(theta, d, a, alpha)
            T = T * A
        return sp.simplify(T)
    
