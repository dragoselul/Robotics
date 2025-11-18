import sympy as sp

from .AlongAxis import AlongAxis


class RotationMatrix:
    def __init__(self):
        pass

    @staticmethod
    def calculate_rotation_matrix(rotations: dict[str, tuple[AlongAxis, float]]):
        result_matrix = sp.eye(3)  # Start with identity matrix

        for axis, (rotation_around, angle) in rotations.items():
            # Extract the axis (string), rotation type (AlongAxis enum), and angle (float)
            print(f"Axis: {axis}")
            print(f"Rotation around: {rotation_around}")
            print(f"Angle: {angle}")

            # Get the rotation matrix for the current axis and angle
            current_rotation = RotationMatrices.get_rotation_matrix_for_axis(axis, angle)

            # Multiply matrices based on rotation type
            if rotation_around == AlongAxis.WORLD:
                # For world rotations, multiply on the right
                result_matrix = current_rotation * result_matrix
            else:  # AlongAxis.CURRENT
                # For current/local rotations, multiply on the left
                result_matrix = result_matrix * current_rotation

        return result_matrix


class RotationMatrices:

    def __init__(self):
        pass

    @staticmethod
    def get_rotation_matrix_for_axis(axis, angle=sp.symbols('theta')):
        match axis:
            case 'X' | 'x':
                return sp.Matrix([
                    [1, 0, 0],
                    [0, sp.cos(angle), (-1) * sp.sin(angle)],
                    [0, sp.sin(angle), sp.cos(angle)]])
            case 'Y' | 'y':
                return sp.Matrix([
                    [sp.cos(angle), 0, sp.sin(angle)],
                    [0, 1, 0],
                    [(-1) * sp.sin(angle), 0, sp.cos(angle)]])
            case 'Z' | 'z':
                return sp.Matrix([
                    [sp.cos(angle), (-1) * sp.sin(angle), 0],
                    [sp.sin(angle), sp.cos(angle), 0],
                    [0, 0, 1]
                ])
            case _:
                raise Exception('Invalid axis')


