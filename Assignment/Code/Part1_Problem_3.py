import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# IMPORT YOUR CONTROLLER
# Assuming robot_controller.py and robot_formulas.py are in the same folder
# We use a mock robot here for simulation purposes.
# In real life, you would import: from robot import Robot
class MockRobot:
    def initialize(self): pass

    def enable_torque(self, ids): pass

    def disable_torque(self, ids): pass

    def move(self, positions): pass

    def read_current_position(self, ids): return {1: 512, 2: 512, 3: 512, 4: 512}

    def close(self): pass


import Robot.kinematics
from Robot.robot_controller import RobotKinematicsController


def get_joint_positions_for_plotting(kin, q):
    """
    Calculates the XYZ position of every joint [Base, Shoulder, Elbow, Wrist, Tool]
    so we can draw the robot as a stick figure.
    """
    q1, q2, q3, q4 = q

    # Link Lengths (must match your RobotFormulas)
    d1 = kin.d1
    a2 = kin.a2
    a3 = kin.a3
    a4 = kin.a4

    # 1. Base (0,0,0)
    p0 = np.array([0, 0, 0])

    # 2. Shoulder (0,0,d1)
    p1 = np.array([0, 0, d1])

    # 3. Elbow
    # Position relative to shoulder
    r2 = a2 * np.cos(q2)
    z2 = a2 * np.sin(q2)
    p2 = np.array([
        r2 * np.cos(q1),
        r2 * np.sin(q1),
        d1 + z2
    ])

    # 4. Wrist
    r3 = r2 + a3 * np.cos(q2 + q3)
    z3 = z2 + a3 * np.sin(q2 + q3)
    p3 = np.array([
        r3 * np.cos(q1),
        r3 * np.sin(q1),
        d1 + z3
    ])

    # 5. End Effector
    r4 = r3 + a4 * np.cos(q2 + q3 + q4)
    z4 = z3 + a4 * np.sin(q2 + q3 + q4)
    p4 = np.array([
        r4 * np.cos(q1),
        r4 * np.sin(q1),
        d1 + z4
    ])

    return [p0, p1, p2, p3, p4]


def execute_circle_movement(controller, center, radius, plane='yz', num_points=37, speed=100):
    """
    Makes the robot physically move in a circle.
    
    Args:
        controller: The RobotKinematicsController instance
        center: [x, y, z] coordinates of the circle center
        radius: Radius of the circle in meters
        plane: 'yz' (front), 'xz' (side), or 'xy' (ground)
        num_points: Number of waypoints to discretize the circle
        speed: Movement speed (if supported by controller)
    """
    print(f"Executing Circle Movement: Center={center}, Radius={radius}, Plane={plane}")
    
    # Generate waypoints
    angles = np.linspace(0, 2 * np.pi, num_points)
    
    for i, phi in enumerate(angles):
        # Calculate target point based on selected plane
        if plane == 'yz':
            # Front plane (facing the robot)
            x = center[0]
            y = center[1] + radius * np.cos(phi)
            z = center[2] + radius * np.sin(phi)
        elif plane == 'xz':
            # Side plane (sagittal)
            x = center[0] + radius * np.cos(phi)
            y = center[1]
            z = center[2] + radius * np.sin(phi)
        elif plane == 'xy':
            # Ground plane
            x = center[0] + radius * np.cos(phi)
            y = center[1] + radius * np.sin(phi)
            z = center[2]
        else:
            raise ValueError("Plane must be 'yz', 'xz', or 'xy'")
        
        print(f"Moving to point {i+1}/{num_points}: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
        # Move to the point
        # Using move_to_position from controller which handles IK
        print("x=", x, "y=", y, "z=", z)
        result = controller.move_to_position(x, y, z)
        
        # Check if result is False (failed) or an array (success)
        if isinstance(result, bool) and not result:
            print(f"⚠ Failed to reach point {i+1}")
        
        # Optional: Add a small delay if needed for stability
        # time.sleep(0.1)


def run_problem3():
    print("=" * 70)
    print("PROBLEM 3: CIRCLE DRAWING (YZ PLANE)")
    print("=" * 70)

    # 1. Setup Controller
    # using MockRobot for simulation visualization
    robot_hw = MockRobot()
    controller = RobotKinematicsController(robot_hw, mock=True)

    # 2. Define Task Parameters
    center = np.array([0.150, 0.0, 0.120])
    radius = 0.032
    num_points = 37

    # 3. Generate Trajectory Points
    angles = np.linspace(0, 2 * np.pi, num_points)
    waypoints = []

    for phi in angles:
        x = center[0]
        y = center[1] + radius * np.cos(phi)
        z = center[2] + radius * np.sin(phi)
        waypoints.append([x, y, z])

    waypoints = np.array(waypoints)

    # 4. Execute & Record
    actual_path = []
    joint_history = []

    print("Computing Inverse Kinematics...")
    for pt in waypoints:
        q = controller.kin.inverse_kinematics(pt)
        if q is not None:
            joint_history.append(q)
            T04, _ = controller.kin.forward_kinematics(q)
            actual_path.append(T04[:3, 3])

    actual_path = np.array(actual_path)

    # 5. VISUALIZATION
    print("Visualizing result...")
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot Target Circle
    ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
            'g--', linewidth=2, label='Target Trajectory')

    # Plot Robot's Computed Path
    ax.scatter(actual_path[:, 0], actual_path[:, 1], actual_path[:, 2],
               c='red', s=20, label='Robot End-Effector')

    # Plot Center
    ax.scatter([center[0]], [center[1]], [center[2]],
               c='k', marker='x', s=100, label='Center')

    # Plot Robot Links for a few frames
    frame_indices = [0, 9, 18, 27]
    colors = ['blue', 'orange', 'purple', 'cyan']
    labels = ['Start (0°)', 'Top (90°)', 'Far (180°)', 'Bottom (270°)']
    joints = None
    for idx, color, label in zip(frame_indices, colors, labels):
        if idx < len(joint_history):
            q = joint_history[idx]

            # --- FIX IS HERE ---
            # Calculate actual XYZ positions of joints
            joints = get_joint_positions_for_plotting(controller.kin, q)

            # Unpack for plotting
            jx = [p[0] for p in joints]
            jy = [p[1] for p in joints]
            jz = [p[2] for p in joints]

            # Plot Links
            ax.plot(jx, jy, jz, 'o-', color=color, linewidth=2, label=f"Robot {label}")
            ax.scatter(jx, jy, jz, color=color, s=30)
    # Aesthetics
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Problem 3: Robot Drawing Circle (4 Key Poses)')
    ax.legend()

    # Aspect Ratio Hack
    try:
        limits = np.array([getattr(ax, f'get_{axis}lim')() for axis in 'xyz'])
        ax.set_box_aspect(np.ptp(limits, axis=1))
    except:
        pass  # Some matplotlib versions don't support box_aspect well

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_problem3()
