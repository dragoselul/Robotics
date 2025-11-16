function K = computeK(jacobian, q_dot, mass, T, I)

    % where jacobian is the new computed jacobian for the COM
    % q_dot = rate of change of pose at the location (column vector)
    % mass = mass of link
    % T = homogenous transform for the COM to the base
    % I = local inertia frame from the exercise definition

    % Extract the rotation
    R = T(1:3, 1:3);

    % Extract the v and w Jacobian components
    Jv = jacobian(1:3, :);
    Jw = jacobian(4:6, :);

    D = mass * (Jv' * Jv) + Jw' * R * I * R' * Jw;

    K = 1/2 * q_dot' * D * q_dot;

end