function jacobian = Jvw(rotation_axis, end_effector, origin)

    % function returns Jv1 Jw1 nx1 vector for a column of the Jacobian in
    % terms of theta1 - theta4
    % ONLY VALID FOR ROTATIONAL JOINTS AND FOR THE SIMULINK MODEL

    Jv = skew(rotation_axis) * (end_effector - origin);
    Jw = rotation_axis;

    jacobian = [Jv' , Jw']';
    
end 