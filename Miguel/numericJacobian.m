function numeric_jacob = numericJacobian(jacobian, pose)

    % calculation of the jacobian for a given pose (this holds for the
    % instant of the pose). The jacobian changes with time, so it keeps
    % being different depending on the pose.

    syms theta1 theta2 theta3 theta4

    numeric_jacob = (double(subs(jacobian, [theta1, theta2, theta3, theta4], [pose(1), pose(2), pose(3), pose(4)])));

end