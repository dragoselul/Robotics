import os
import sys

from Assignment.Code.util import AlongAxis
from Assignment.Code.util import HomogeneousTransformMatrix

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
import sympy as sp

"""
Determine a homogeneous transformation matrix H that represents a rotation
with an angle α about the world x-axis, followed by a translation with a length b
along the world z-axis, followed by a rotation φ about the current y-axis.
"""

if __name__ == "__main__":
    rotation_X = HomogeneousTransformMatrix.create_homogeneous_transform_rotation('X', sp.symbols('alpha'))
    translation_Z = HomogeneousTransformMatrix.create_homogeneous_transform_translation(0, 0, sp.symbols('b'))
    rotation_Y = HomogeneousTransformMatrix.create_homogeneous_transform_rotation('Y', sp.symbols('phi'))

    matrices = [(AlongAxis.WORLD, rotation_X),
                (AlongAxis.WORLD, translation_Z),
                (AlongAxis.CURRENT, rotation_Y)]

    sp.pprint(HomogeneousTransformMatrix.composite_homogeneous_transformation_matrix(matrices), use_unicode=True)