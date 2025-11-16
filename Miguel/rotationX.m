function matrix = rotationX(x)
% rotationMatrixH  Homogeneous rotation matrices (4x4).
%
%   x = rotation about X axis (rad)


    matrix = [1, 0,       0,      0;
              0, cos(x), -sin(x), 0;
              0, sin(x),  cos(x), 0;
              0, 0,       0,       1];

end
