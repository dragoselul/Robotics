import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from Assignment.Code import util
import sympy as sp

"""
What is the rotation matrix for a rotation of 30◦ about the world z-axis, followed
by a rotation of 60◦ about the world x-axis, followed by a rotation of 90◦ about
the world y-axis?
"""

"""
So:
1. Rotate about z-axis by 30 degrees
2. Rotate about x-axis by 60 degrees
3. Rotate about y-axis by 90 degrees

thus the rotation matrix is:
R = R_y(90) * R_x(60) * R_z(30)
"""

if __name__ == "__main__":
    rotations = {
        'Z': (util.AlongAxis.WORLD, sp.rad(30)),
        'X': (util.AlongAxis.WORLD, sp.rad(60)),
        'Y': (util.AlongAxis.WORLD, sp.rad(90))
    }
    R = util.RotationMatrix.calculate_rotation_matrix(rotations)
    sp.pprint(R.evalf(3), use_unicode=True)