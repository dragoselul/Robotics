import sympy as sp


# A_i
class HomogeneousTransformMatrix:
    def __init__(self, theta = sp.symbols('theta'), d = sp.symbols('d'), a = sp.symbols('a'), alpha = sp.symbols('alpha')):
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha

        # ----- Classic DH single-step transform -----
    def get_A_dh(self):
        cos_alpha, sin_alpha = sp.cos(self.alpha), sp.sin(self.alpha)
        cos_theta, sin_theta = sp.cos(self.theta), sp.sin(self.theta)
        return sp.Matrix([
            [cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, self.a*cos_theta],
            [sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, self.a*sin_theta],
            [0,         sin_alpha,         cos_alpha,        self.d],
            [ 0,      0,      0,    1]
        ])

class ForwardKinematics:
    def __init__(self, dh_table):
        self.dh_table = dh_table

    def forward_kinematics_from_dh(self):
        T = sp.eye(4)
        for (a, alpha, d, theta) in self.dh_table:
            A = HomogeneousTransformMatrix(theta, d, a, alpha).get_A_dh()
            T = T * A
        return sp.simplify(T)
    
