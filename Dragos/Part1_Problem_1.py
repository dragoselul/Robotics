import util.AssignmentFunctionsSymbolic as util
import sympy as sp

if __name__ == "__main__":
    T01 = util.DH(sp.symbols('theta_1'), 50, 0, sp.pi/2)
    T12 = util.DH(sp.symbols('theta_2')+ sp.pi/2, 0, 93, 0)
    T23 = util.DH(sp.symbols('theta_3'), 0, 93, 0)
    T34 = util.DH(sp.symbols('theta_4'), 0, 50, 0)
    T03 = util.forward_kinematics([T01, T12, T23])
    T04 = util.forward_kinematics([T01, T12, T23, T34])
    T35 = util.translation_matrix(35,45,0)
    T05 = util.forward_kinematics([T03, T35])

    sp.pprint(T04.evalf(3), use_unicode=True)
    sp.pprint(T05.evalf(3), use_unicode=True)
