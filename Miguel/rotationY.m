function matrix = rotationY(y)
% rotationMatrixH  Homogeneous rotation matrices (4x4).
%
%   y = rotation about Y axis (rad)

    
    matrix = [ cos(y), 0, sin(y), 0;
                0,       1, 0,       0;
                -sin(y), 0, cos(y), 0;
                0,       0, 0,       1];

end