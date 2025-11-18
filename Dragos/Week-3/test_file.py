import sympy as sp

def get_A_dh(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,             sp.sin(alpha),                sp.cos(alpha),               d],
        [0, 0, 0, 1]
    ])

dh_table = [
    (sp.symbols('q1'), 0, 1, 0),
    (sp.symbols('q2'), 0, 1, 0),
]
T = sp.eye(4)
for (theta, d, a, alpha) in dh_table:
    A = get_A_dh(theta, d, a, alpha)
    T = T * A


for i in range(4):
    print(T[i,:], "\n")