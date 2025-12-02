import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from Dragos import util
import sympy as sp

"""
What is the rotation matrix for a rotation φ about the world x-axis, followed by
a rotation ψ about the current z-axis, followed by a rotation θ about the world
y-axis?
"""

if __name__ == "__main__":
    rotations = {
        'X': (util.AlongAxis.WORLD, sp.symbols("phi")),
        'Z': (util.AlongAxis.CURRENT, sp.symbols("psi")),
        'Y': (util.AlongAxis.WORLD, sp.symbols("theta"))
    }
    R = util.RotationMatrix.calculate_rotation_matrix(rotations)
    sp.pprint(R, use_unicode=True)
