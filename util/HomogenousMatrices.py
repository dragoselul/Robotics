from util.RotationMatrices import *
from util.AlongAxis import *
import sympy as sp

class HomogeneousTransformMatrix:
    def __init__(self):
        pass

    @staticmethod
    def create_homogeneous_transform_rotation(axis, angle=sp.symbols('theta')):
        R = RotationMatrices.get_rotation_matrix_for_axis(axis, angle)
        T = sp.eye(4)
        T[:3, :3] = R
        return T

    @staticmethod
    def create_homogeneous_transform_translation(x=0, y=0, z=0):
        T = sp.eye(4)
        T[0,0]=1
        T[1,1]=1
        T[2,2]=1
        T[3,3]=1
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    @staticmethod
    def composite_homogeneous_transformation_matrix(matrices: list[tuple[AlongAxis, sp.Matrix]]):
        H = sp.eye(4)
        for rotation_type, matrix in matrices:
            if rotation_type == AlongAxis.WORLD:
                #pre multiplication if rotation around world
                H = matrix * H
            elif rotation_type == AlongAxis.CURRENT:
                #post multiplication if rotation around current
                H = H * matrix
            else:
                raise Exception("Invalid rotation type")
        return H