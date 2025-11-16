function matrix = rotationZ(z)
% rotationMatrixH  Homogeneous rotation matrices (4x4).
%   z = rotation about Z axis (rad)
    matrix = [cos(z), -sin(z), 0, 0;
              sin(z),  cos(z), 0, 0;
              0,        0,       1, 0;
              0,        0,       0, 1];

end
