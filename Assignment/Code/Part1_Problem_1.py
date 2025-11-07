import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

import util
import sympy as sp

if __name__ == "__main__":
    dh_table_for_t0_4 = [
        (sp.symbols('theta_1'), 50, 0, sp.pi/2),  # theta, d, a, alpha
        (sp.symbols('theta_2')+ sp.pi/2, 0, 93, 0),
        (sp.symbols('theta_3'), 0, 93, 0),
        (sp.symbols('theta_4'), 0, 50, 0)]

    dh_table_for_t0_5 = [
        (sp.symbols('theta_1'), 50, 0, sp.pi/2),  # theta, d, a, alpha
        (sp.symbols('theta_2')+ sp.pi/2, 0, 93, 0),
        (sp.symbols('theta_3'), 0, 93, 0),
        (sp.symbols('theta_4'), 0, 50, 0),
        (sp.symbols('theta_5'), 0, -15, 0)]

    T0_4 = util.ForwardKinematics.forward_kinematics_from_dh(dh_table_for_t0_4)
    T0_5 = util.ForwardKinematics.forward_kinematics_from_dh(dh_table_for_t0_5)
    sp.pprint(T0_4.evalf(3), use_unicode=True)
    sp.pprint(T0_5.evalf(3), use_unicode=True)

