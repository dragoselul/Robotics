import numpy as np

def cos_theorem(a, b, c):
    """
    Law of cosines:
    returns angle opposite side a in a triangle with sides a, b, c
    """
    return np.arccos((b**2 + c**2 - a**2) / (2 * b * c))


def IK03(O, elbow_up = True):
    """
    Compute IK for frame 0→3 using trigonometry.
    O = [x, y, z] end-effector position in frame 0 (3×1 array-like)
    Returns a 2×3 numpy array:
        [theta1, theta2_up,   theta3_up  ]
        [theta1, theta2_down, theta3_down]
    """
    x, y, z = O

    # --- theta1 ---
    theta1 = np.arctan2(y, x)

    # --- compute r and s ---
    r = np.hypot(x, y)
    s = z - 50

    psi = np.arctan2(s, r)

    # using correct link lengths (93 and 93)
    beta = cos_theorem(np.hypot(r, s), 93, 93)

    # --- Compute phi ---
    phi = np.arctan2(93 * np.sin(np.pi - beta),
                     93 * np.cos(np.pi - beta) + 93)

    # --- theta2 up/down ---
    theta2_down = -np.pi/2 + psi - phi
    theta2_up   = -np.pi/2 + psi + phi

    # --- theta3 up/down ---
    theta3_down = np.pi - beta
    theta3_up   = -theta3_down


    if elbow_up is True:
        return np.array(
        [theta1, theta2_up,   theta3_up])
    else:
        return np.array([theta1, theta2_down, theta3_down])

# ===== TRANSFORMATIONS ====
# not symbolic this time 

def translation_matrix_np(x, y, z):
    return np.array([
        [1., 0., 0., x],
        [0., 1., 0., y],
        [0., 0., 1., z],
        [0., 0., 0., 1.],
    ])

def rotation_x_np(x):
    c, s = np.cos(x), np.sin(x)
    return np.array([
        [1., 0., 0., 0.],
        [0.,  c, -s, 0.],
        [0.,  s,  c, 0.],
        [0., 0., 0., 1.],
    ])

def rotation_y_np(y):
    c, s = np.cos(y), np.sin(y)
    return np.array([
        [ c, 0.,  s, 0.],
        [0., 1., 0., 0.],
        [-s, 0.,  c, 0.],
        [0., 0., 0., 1.],
    ])

def rotation_z_np(z):
    c, s = np.cos(z), np.sin(z)
    return np.array([
        [ c, -s, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

def DH_np(theta, dz, dx, alpha):
    return (
        rotation_z_np(theta)
        @ translation_matrix_np(0., 0., dz)
        @ translation_matrix_np(dx, 0., 0.)
        @ rotation_x_np(alpha)
    )

def transformation(theta1, theta2, theta3, theta4):
    """
    Compute all homogeneous transforms from base (0) to frame 4 (stylus tip).

    Returns a dict with:
      - "T01"   : 0 -> 1
      - "T02"   : 0 -> 2
      - "T03"   : 0 -> 3
      - "T04"   : 0 -> 4 (end of stylus)
    """

    # Relative DH transforms (same as MATLAB)
    T01     = DH_np(theta1, 50.0, 0.0, np.pi/2)
    T12     = DH_np(theta2 + np.pi/2, 0.0, 93.0, 0.0)
    T23     = DH_np(theta3, 0.0, 93.0, 0.0)
    T34     = DH_np(theta4, 0.0, 50.0, 0.0)

    # Absolute transforms wrt original base
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34  # <-- end of stylus
    T05     = T03 @ translation_matrix_np(35, 45, 0)


    return {
        "T01": T01,
        "T02": T02,
        "T03": T03,
        "T04": T04,
        "T05": T05
    }

def IK4(X_des, O_des, elbow_up=True):
    """
    X_des (ORIENTATION) : desired end-effector x-axis (3,) in frame {0}
    O_des (POSITION) : desired end-effector origin o_4 (3,) in frame {0}
    elbow_up : choose IK03 branch (True = elbow up, False = elbow down)
    base_offset_deg : base rotation offset (same as in transformations)

    Returns: [theta1, theta2, theta3, theta4]
    """

    L4 = 50.0  # link-4 length along x_4

    # Ensure numpy arrays of correct shape
    X_des = np.asarray(X_des, dtype=float).reshape(3)
    O_des = np.asarray(O_des, dtype=float).reshape(3)

    # 1) Normalize desired x_4
    Xhat = X_des / np.linalg.norm(X_des)

    # 2) Wrist center o3 = O_des - L4 * Xhat
    o3 = O_des - L4 * Xhat

    # 3) Solve 2R IK for {0} -> o3
    t1, t2, t3 = IK03(o3, elbow_up=elbow_up)

    # 4) Build numeric R03 for that branch using our FK
    # theta4 can be 0.0 here because T03 does not depend on theta4
    Ts = transformation(t1, t2, t3, 0.0)
    T03 = Ts["T03"]
    R03 = T03[:3, :3]

    # 5) Feasibility: x4 must be ⟂ z3
    z3 = R03[:, 2]
    if abs(np.dot(Xhat, z3)) > 1e-10:
        raise ValueError(
            "Pose not achievable: desired x4 not perpendicular to z3 at that wrist position."
        )

    # 6) θ4 from projections on x3, y3
    x3 = R03[:, 0]
    y3 = R03[:, 1]
    c4 = np.dot(Xhat, x3)
    s4 = np.dot(Xhat, y3)
    t4 = np.arctan2(s4, c4)

    return np.array([t1, t2, t3, t4])

# Desired orientation (x-axis of end-effector in {0})
X_des = np.array([1.0, 0.0, 0.0])  # e.g. pointing along global x

# Desired position of stylus tip in {0}
O_des = np.array([182, 0, 125])

angles = IK4(X_des, O_des, elbow_up=True)
print("θ1..θ4:", angles)


