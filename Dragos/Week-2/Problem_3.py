import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
import util
import sympy as sp

"""
Find another sequence of rotations that is different from Prob. 2, but which
results in the same rotation matrix
"""

# in the problem-2 we did Z*X*Y se we need to do rotations that achieve the same equation

if __name__ == "__main__":
    rotations = {
        'Z': (util.AlongAxis.WORLD, sp.symbols("psi")),
        'X': (util.AlongAxis.WORLD, sp.symbols("phi")),
        'Y': (util.AlongAxis.WORLD, sp.symbols("theta"))
    }
    R = util.RotationMatrix.calculate_rotation_matrix(rotations)
    sp.pprint(R, use_unicode=True)