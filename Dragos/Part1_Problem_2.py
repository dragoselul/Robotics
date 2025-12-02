import util.AssignmentFunctionsSymbolic as util
import sympy as sp

""" Problem 2

First calculate the joint 3 position (frame 3) 
From frame 3, both a rotation and transformation are applied to attain
x04 and o04

Requirements

 Theta1 = frame 0
 Theta2 = frame 1
 Theta3 = frame 2
 Theta4 = frame 3
 Link1 = 50mm
 Link2 = 93mm
 Link3 = 93mm
 Link4 = 50mm
 """

if __name__ == "__main__":
    T01 = util.DH(sp.symbols('theta_1'), 50, 0, sp.pi/2)
    T12 = util.DH(sp.symbols('theta_2')+ sp.pi/2, 0, 93, 0)
    T23 = util.DH(sp.symbols('theta_3'), 0, 93, 0)
    T34 = util.DH(sp.symbols('theta_4'), 0, 50, 0)
    T03 = util.forward_kinematics([T01, T12, T23])
    T04 = util.forward_kinematics([T01, T12, T23, T34])
    T35 = util.translation_matrix(35,45,0)
    T05 = util.forward_kinematics([T03, T35])

    # Define a desired position to test inverse kinematics
    # Example: origin at (100, 100, 150)
    origin_3_desired = sp.Matrix([100, 100, 150])
    
    # Call inverse_kinematics0_3 with the desired position
    angles = util.inverse_kinematics0_3(origin_3_desired)
    
    # Extract angles from the solution (first row, elbow-down configuration)
    theta_1 = angles[0, 0]
    theta_2 = angles[0, 1]
    theta_3 = angles[0, 2]
    
    # theta_4 can be set to 0 or another value as needed
    theta_4 = 0
    
    result = util.check4(T04, theta_1, theta_2, theta_3, theta_4)
    sp.pprint(result, use_unicode=True)