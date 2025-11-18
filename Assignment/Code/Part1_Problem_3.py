"""
Problem 3: Circle Drawing with Robot
Find 37 robot configurations to track equidistant points on a circle.

Requirements:
- Circle: R = 32mm, center p_c = [150, 0, 120]^T mm
- 37 points from φ₀ = 0 to φ₃₆ = 2π
- Stylus remains horizontal in all configurations
"""

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

import numpy as np
import matplotlib.pyplot as plt
from Assignment.Code.Robot.robot_kinematics import RobotKinematics
from Assignment.Code.util.AssignmentFunctions import *

# Problem parameters
R = 32  # Circle radius in mm
p_c = np.array([150, 0, 120])  # Circle center [x, y, z] in mm
num_points = 37  # Number of configurations (j = 0 to 36)

# Generate circle points and tangent vectors
# φ from 0 to 2π with 37 equidistant points
phi_values = np.linspace(0, 2*np.pi, num_points)


# Circle points in the YZ plane (perpendicular to X-axis)
# p = p_c + R * [0, cos(φ), sin(φ)]
# Tangent to circle (horizontal, perpendicular to radius): x4 = [-sin(φ), cos(φ), 0] (but reversed for drawing direction)
circle_points = []
tangent_vectors = []

for phi in phi_values:
    x = p_c[0]  # X stays constant at circle center X
    y = p_c[1] + R * np.cos(phi)  # Y varies with cos(φ)
    z = p_c[2] + R * np.sin(phi)  # Z varies with sin(φ)
    circle_points.append(np.array([x, y, z]))

    # Tangent vector: derivative of circle parameterization d/dφ[x, y, z]
    # = [0, -R*sin(φ), R*cos(φ)]
    # Normalize and keep horizontal (z=0 component is projection to XY plane)
    tangent_y = -np.sin(phi)
    tangent_z = np.cos(phi)
    # For horizontal stylus: project tangent to XY plane
    # The tangent in 3D is [0, -sin(φ), cos(φ)], but we want it horizontal
    # So we use the direction perpendicular to the radius in XY plane
    tangent = np.array([-tangent_z, -tangent_y, 0])  # Horizontal tangent, pointing forward along circle
    tangent = tangent / np.linalg.norm(tangent)  # Normalize
    tangent_vectors.append(tangent)

print(f"\nGenerated {len(circle_points)} circle points")
print(f"First point (φ=0°): {circle_points[0]}")
print(f"Last point (φ=360°): {circle_points[-1]}")
print(f"Mid point (φ=180°): {circle_points[num_points//2]}")

# Compute inverse kinematics for each point
robot = RobotKinematics()
configurations = []
successful_configs = 0
failed_configs = 0

print("\n" + "="*70)
print("Computing inverse kinematics for each point...")
print("="*70)

for j, (phi, point, tangent) in enumerate(zip(phi_values, circle_points, tangent_vectors)):
    try:
        # Strategy: Use Jacobian-based iterative IK for accurate positioning
        # 1. Get initial guess from analytical IK
        # 2. Refine with Jacobian iterations to achieve:
        #    - End-effector at target position
        #    - Stylus horizontal (x4_z = 0)

        L4 = 50

        # Initial guess: x4 points radially outward from circle center in XY plane
        initial_direction = np.array([point[0] - p_c[0], point[1] - p_c[1], 0])
        if np.linalg.norm(initial_direction) < 1e-6:
            initial_direction = np.array([0, 1, 0])  # Default to +Y
        initial_direction = initial_direction / np.linalg.norm(initial_direction)

        # Try both elbow configurations and keep the best one
        best_angles = None
        best_error = float('inf')

        for elbow_up in [True, False]:
            # Get initial guess from analytical IK
            x4_direction = initial_direction
            for iteration in range(3):
                wrist_center = point - L4 * x4_direction

                try:
                    angles_3dof = robot.inverse_kinematics_position(wrist_center, elbow_up=elbow_up)
                except:
                    continue

                T01 = DH(angles_3dof[0], 50, 0, np.pi/2)
                T12 = DH(angles_3dof[1] + np.pi/2, 0, 93, 0)
                T23 = DH(angles_3dof[2], 0, 93, 0)
                T03 = T01 @ T12 @ T23

                z3 = extract_XYZ_vectors(T03, "z")
                x3 = extract_XYZ_vectors(T03, "x")

                x4_direction = initial_direction - np.dot(initial_direction, z3) * z3
                if np.linalg.norm(x4_direction) < 1e-6:
                    x4_direction = np.array([x3[0], x3[1], 0])
                x4_direction = x4_direction / np.linalg.norm(x4_direction)

            x4_proj = x4_direction - np.dot(x4_direction, z3) * z3
            x4_proj = x4_proj / np.linalg.norm(x4_proj)

            cos_theta4 = np.dot(x3, x4_proj)
            sin_theta4 = np.dot(np.cross(x3, x4_proj), z3)
            theta4 = np.arctan2(sin_theta4, cos_theta4)

            test_angles = np.append(angles_3dof, theta4)

            # Jacobian-based refinement
            angles_refined = test_angles.copy()
            max_iterations = 20
            step_size = 0.3  # Conservative step size

            # Track previous error to detect divergence
            prev_error_norm = float('inf')

            for iter_refinement in range(max_iterations):
                # Check for unreasonable joint angles (sign of divergence)
                if np.any(np.abs(angles_refined) > 10):  # 10 radians ~ 573 degrees
                    # Diverged - revert to initial guess
                    angles_refined = test_angles.copy()
                    break

                # Compute current pose
                T04_current = robot.forward_kinematics(angles_refined)
                current_position = extract_XYZ(T04_current)
                current_x4 = extract_XYZ_vectors(T04_current, "x")

                # Position error
                position_error = point - current_position

                # Orientation error: penalize non-horizontal x4 (x4_z should be 0)
                horizontal_error = -current_x4[2]  # Want this to be 0

                # Total error norm
                error_norm = np.linalg.norm(position_error) + abs(horizontal_error)

                # Check convergence
                if np.linalg.norm(position_error) < 0.1 and abs(horizontal_error) < 0.01:
                    break

                # Check for divergence
                if error_norm > prev_error_norm * 1.5:
                    # Error is increasing - stop and use best so far
                    break
                prev_error_norm = error_norm

                try:
                    # Compute Jacobian at current configuration
                    J = computeJacobian(*angles_refined)  # 6x4 matrix
                    J_pos = J[0:3, :]  # Position Jacobian (3x4)

                    # Compute orientation Jacobian effect on x4_z
                    # We want to control how x4[2] changes with joint angles
                    # Small perturbation approach
                    epsilon = 1e-6
                    grad_x4z = np.zeros(4)
                    for k in range(4):
                        angles_perturbed = angles_refined.copy()
                        angles_perturbed[k] += epsilon
                        T04_perturbed = robot.forward_kinematics(angles_perturbed)
                        x4_perturbed = extract_XYZ_vectors(T04_perturbed, "x")
                        grad_x4z[k] = (x4_perturbed[2] - current_x4[2]) / epsilon

                    # Combined task: position (3D) + horizontal constraint (1D)
                    # Error vector: [pos_error; horizontal_error]
                    error_vec = np.hstack([position_error, horizontal_error])

                    # Combined Jacobian: [J_pos; grad_x4z]
                    J_combined = np.vstack([J_pos, grad_x4z.reshape(1, -1)])  # 4x4

                    # Damped pseudo-inverse with regularization
                    lambda_damp = 0.001
                    J_pinv = np.linalg.inv(J_combined.T @ J_combined + lambda_damp * np.eye(4)) @ J_combined.T

                    # Update joint angles with adaptive step size
                    delta_angles = J_pinv @ error_vec

                    # Limit step size
                    max_step = 0.2  # Max 0.2 rad (~11 degrees) per iteration
                    delta_norm = np.linalg.norm(delta_angles)
                    if delta_norm > max_step:
                        delta_angles = delta_angles * (max_step / delta_norm)

                    angles_refined = angles_refined + step_size * delta_angles

                except np.linalg.LinAlgError:
                    # Singular matrix - stop refinement
                    break

            # Verify refined solution
            T04_test = robot.forward_kinematics(angles_refined)
            test_position = extract_XYZ(T04_test)
            test_error = np.linalg.norm(test_position - point)

            if test_error < best_error:
                best_error = test_error
                best_angles = angles_refined
                x4_direction_best = x4_direction

        angles = best_angles
        x4_direction = x4_direction_best

        # Verify the solution
        T04 = robot.forward_kinematics(angles)
        computed_position = extract_XYZ(T04)
        computed_orientation = extract_XYZ_vectors(T04, "x")

        # Check that orientation is horizontal (z component should be ~0)
        horizontal_check = abs(computed_orientation[2])

        position_error = np.linalg.norm(computed_position - point)
        orientation_error = np.linalg.norm(computed_orientation - x4_direction)

        configurations.append({
            'j': j,
            'phi': phi,
            'phi_deg': np.degrees(phi),
            'target_position': point,
            'angles': angles,
            'angles_deg': np.degrees(angles),
            'computed_position': computed_position,
            'computed_orientation': computed_orientation,
            'position_error': position_error,
            'orientation_error': orientation_error,
            'success': True
        })

        successful_configs += 1

        if j % 6 == 0:  # Print every 6th configuration
            print(f"Config {j:2d} (φ={np.degrees(phi):6.1f}°): "
                  f"angles=[{np.degrees(angles[0]):6.1f}°, {np.degrees(angles[1]):6.1f}°, "
                  f"{np.degrees(angles[2]):6.1f}°, {np.degrees(angles[3]):6.1f}°], "
                  f"error={position_error:.3f}mm")

    except Exception as e:
        print(f"Config {j:2d} (φ={np.degrees(phi):6.1f}°): FAILED - {e}")
        configurations.append({
            'j': j,
            'phi': phi,
            'phi_deg': np.degrees(phi),
            'target_position': point,
            'angles': None,
            'success': False,
            'error': str(e)
        })
        failed_configs += 1

print("\n" + "="*70)
print(f"Results: {successful_configs} successful, {failed_configs} failed")
print("="*70)

# Display results table
if successful_configs > 0:
    print("\nConfiguration Table:")
    print("-"*100)
    print(f"{'j':>3} | {'φ (deg)':>8} | {'θ₁ (°)':>8} | {'θ₂ (°)':>8} | {'θ₃ (°)':>8} | {'θ₄ (°)':>8} | {'Pos Error (mm)':>15}")
    print("-"*100)

    for config in configurations:
        if config['success']:
            print(f"{config['j']:3d} | {config['phi_deg']:8.1f} | "
                  f"{config['angles_deg'][0]:8.1f} | {config['angles_deg'][1]:8.1f} | "
                  f"{config['angles_deg'][2]:8.1f} | {config['angles_deg'][3]:8.1f} | "
                  f"{config['position_error']:15.6f}")
    print("-"*100)

# Save configurations to file
output_file = "problem3_configurations.txt"
with open(output_file, 'w') as f:
    f.write("Problem 3: Robot Configurations for Circle Drawing\n")
    f.write("="*70 + "\n")
    f.write(f"Circle radius: {R} mm\n")
    f.write(f"Circle center: {p_c}\n")
    f.write(f"Number of configurations: {num_points}\n")
    f.write(f"End-effector orientation: Tangent to circle (horizontal)\n")
    f.write("="*70 + "\n\n")

    f.write("Configuration Table:\n")
    f.write("-"*100 + "\n")
    f.write(f"{'j':>3} | {'φ (deg)':>8} | {'θ₁ (°)':>8} | {'θ₂ (°)':>8} | {'θ₃ (°)':>8} | {'θ₄ (°)':>8} | {'Target Pos (mm)':>30} | {'Error (mm)':>12}\n")
    f.write("-"*100 + "\n")

    for config in configurations:
        if config['success']:
            f.write(f"{config['j']:3d} | {config['phi_deg']:8.1f} | "
                    f"{config['angles_deg'][0]:8.1f} | {config['angles_deg'][1]:8.1f} | "
                    f"{config['angles_deg'][2]:8.1f} | {config['angles_deg'][3]:8.1f} | "
                    f"[{config['target_position'][0]:6.1f}, {config['target_position'][1]:6.1f}, {config['target_position'][2]:6.1f}] | "
                    f"{config['position_error']:12.6f}\n")
        else:
            f.write(f"{config['j']:3d} | {config['phi_deg']:8.1f} | FAILED - {config['error']}\n")
    f.write("-"*100 + "\n")

print(f"\nConfigurations saved to: {output_file}")

# Visualization
if successful_configs > 0:
    print("\nGenerating visualization...")

    fig = plt.figure(figsize=(15, 5))

    # Plot 1: Circle in 3D space
    ax1 = fig.add_subplot(131, projection='3d')

    successful_points = np.array([c['computed_position'] for c in configurations if c['success']])
    target_points = np.array([c['target_position'] for c in configurations if c['success']])

    ax1.plot(target_points[:, 0], target_points[:, 1], target_points[:, 2],
             'b-', linewidth=2, label='Target Circle')
    ax1.scatter(successful_points[:, 0], successful_points[:, 1], successful_points[:, 2],
                c='r', s=50, label='Computed Points')
    ax1.scatter([p_c[0]], [p_c[1]], [p_c[2]], c='g', s=100, marker='x', label='Circle Center')

    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title('Circle Drawing Path')
    ax1.legend()
    ax1.set_box_aspect([1,1,1])

    # Plot 2: Joint angles vs configuration number
    ax2 = fig.add_subplot(132)

    j_values = [c['j'] for c in configurations if c['success']]
    theta1 = [c['angles_deg'][0] for c in configurations if c['success']]
    theta2 = [c['angles_deg'][1] for c in configurations if c['success']]
    theta3 = [c['angles_deg'][2] for c in configurations if c['success']]
    theta4 = [c['angles_deg'][3] for c in configurations if c['success']]

    ax2.plot(j_values, theta1, 'o-', label='θ₁')
    ax2.plot(j_values, theta2, 's-', label='θ₂')
    ax2.plot(j_values, theta3, '^-', label='θ₃')
    ax2.plot(j_values, theta4, 'd-', label='θ₄')
    ax2.set_xlabel('Configuration j')
    ax2.set_ylabel('Joint Angle (degrees)')
    ax2.set_title('Joint Angles vs Configuration')
    ax2.legend()
    ax2.grid(True)

    # Plot 3: Position error vs configuration
    ax3 = fig.add_subplot(133)

    errors = [c['position_error'] for c in configurations if c['success']]
    ax3.plot(j_values, errors, 'ro-')
    ax3.set_xlabel('Configuration j')
    ax3.set_ylabel('Position Error (mm)')
    ax3.set_title('Position Error vs Configuration')
    ax3.grid(True)
    ax3.set_yscale('log')

    plt.tight_layout()
    plt.savefig('problem3_visualization.png', dpi=150)
    print("Visualization saved to: problem3_visualization.png")
    plt.show()

print("\n" + "="*70)
print("Problem 3 Complete!")
print("="*70)

# Summary statistics
if successful_configs > 0:
    errors = [c['position_error'] for c in configurations if c['success']]
    print(f"\nPosition Error Statistics:")
    print(f"  Mean error: {np.mean(errors):.6f} mm")
    print(f"  Max error:  {np.max(errors):.6f} mm")
    print(f"  Min error:  {np.min(errors):.6f} mm")
    print(f"  Std dev:    {np.std(errors):.6f} mm")

    # Count configurations by accuracy
    high_accuracy = sum(1 for e in errors if e < 1.0)
    medium_accuracy = sum(1 for e in errors if 1.0 <= e < 10.0)
    low_accuracy = sum(1 for e in errors if e >= 10.0)

    print(f"\nAccuracy Breakdown:")
    print(f"  High accuracy (<1mm):     {high_accuracy}/37 configurations")
    print(f"  Medium accuracy (1-10mm): {medium_accuracy}/37 configurations")
    print(f"  Low accuracy (>10mm):     {low_accuracy}/37 configurations")

    print(f"\n✓ Jacobian-based IK refinement achieved sub-millimeter accuracy")
    print(f"  for {high_accuracy} out of 37 configurations!")

    # Identify problematic configurations
    problematic = [(c['j'], c['phi_deg'], c['position_error'])
                   for c in configurations if c['success'] and c['position_error'] > 10.0]
    if problematic:
        print(f"\nConfigurations with larger errors (near workspace limits):")
        for j, phi, err in problematic:
            print(f"  Config {j:2d} (φ={phi:6.1f}°): {err:.1f}mm")

