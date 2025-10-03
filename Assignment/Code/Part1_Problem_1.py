import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

import util
import sympy as sp

if __name__ == "__main__":
    dh_table_for_t0_4 = [
        (0, 90, 50, 0),  # a, alpha, d, theta
        (93, 0, 0, 90),
        (93, 0, 0, 0),
        (50, 0, 0, 0)]

    dh_table_for_t0_5 = [(0, 90, 50, 0),  # a, alpha, d, theta
        (93, 0, 0, 90),
        (93, 0, 0, 0),
        (50, 0, 0, 0),
        (0, 0, 15, 0)]
    fk = util.ForwardKinematics(dh_table_for_t0_4)
    fk = util.ForwardKinematics(dh_table_for_t0_5)
    T0_4 = fk.forward_kinematics_from_dh()
    T0_5 = fk.forward_kinematics_from_dh()
    sp.pprint(T0_4.evalf(3), use_unicode=True)
    sp.pprint(T0_5.evalf(3), use_unicode=True)

