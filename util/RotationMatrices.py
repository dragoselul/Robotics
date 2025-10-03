import sympy as sp


# TODO: not finished yet, need to make it a class
def rotation_matrices(theta_X, theta_Y, theta_Z):
    # Rotation matrices
    Rx = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(theta_X), (-1)*sp.sin(theta_X)],
        [0, sp.sin(theta_X),  sp.cos(theta_X)]
    ])
    Rx = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(theta_X), (-1)*sp.sin(theta_X)],
        [0, sp.sin(theta_X),  sp.cos(theta_X)]
    ])
    Ry = sp.Matrix([
        [sp.cos(theta_Y), 0, sp.sin(theta_Y)],
        [0, 1, 0],
        [(-1)*sp.sin(theta_Y), 0, sp.cos(theta_Y)]
    ])
    Rz = sp.Matrix([
        [sp.cos(theta_Z), (-1)*sp.sin(theta_Z), 0],
        [sp.sin(theta_Z),  sp.cos(theta_Z), 0],
        [0, 0, 1]
    ])
# Example: 90°, 60°, 30° in radians
result = rotation_matrices(sp.pi/2, sp.pi/3, sp.pi/6)
sp.pprint(result)   # pretty print with sqrt fractions
