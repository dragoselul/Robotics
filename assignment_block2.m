%% Functions

function S = skew(v)
    % Computes the skew-symmetric matrix of a 3D vector
    % Input:  v = [vx; vy; vz]
    % Output: S = [v]_x

    S = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0 ];
end


function matrix = translationMatrix(x, y, z)
    % translationMatrix returns the homogeneous transformation matrix
    % for a translation along x, y, z axes.
    %
    % x = translation along X axis
    % y = translation along Y axis
    % z = translation along Z axis

    % homogeneous 4x4 translation matrix
    matrix = [1, 0, 0, x;
              0, 1, 0, y;
              0, 0, 1, z;
              0, 0, 0, 1];
end


function matrix = rotationX(x)
% rotationMatrixH  Homogeneous rotation matrices (4x4).
%
%   x = rotation about X axis (rad)


    matrix = [1, 0,       0,      0;
              0, cos(x), -sin(x), 0;
              0, sin(x),  cos(x), 0;
              0, 0,       0,       1];

end

function matrix = rotationY(y)
% rotationMatrixH  Homogeneous rotation matrices (4x4).
%
%   y = rotation about Y axis (rad)

    
    matrix = [ cos(y), 0, sin(y), 0;
                0,       1, 0,       0;
                -sin(y), 0, cos(y), 0;
                0,       0, 0,       1];

end

function matrix = rotationZ(z)
% rotationMatrixH  Homogeneous rotation matrices (4x4).
%   z = rotation about Z axis (rad)
    matrix = [cos(z), -sin(z), 0, 0;
              sin(z),  cos(z), 0, 0;
              0,        0,       1, 0;
              0,        0,       0, 1];

end


function point = extractXYZ(A)

    matrix.size = size(A);

    if sum(size(A) == [matrix.size(1) matrix.size(1)]) < 2
        
        error('Matrix is not square')

    end

    point = A(1:end-1, matrix.size(1));

end 


function vector = extractXYZvectors(A, vector_name)

    % where A is the H matrix

    matrix.size = size(A);

    if sum(size(A) == [matrix.size(1) matrix.size(1)]) < 2
        
        error('Matrix is not square')

    end

    if vector_name == 1 %x
        vector = A(1:end-1, 1);
    elseif vector_name == 2 %y
        vector = A(1:end-1, 2);
    elseif vector_name == 3 %z
        vector = A(1:end-1, 3);
    else
        error("Incorrect vector, give range from 1-3")
    end

end 


function transformation = DH(theta, dz, dx, alpha)
    
    transformation = rotationZ(theta) * translationMatrix(0, 0, dz) * translationMatrix(dx, 0, 0) * rotationX(alpha);


end

function angle = cosTheorem(a, b, c)

    % where a, b, c are the sides of the triangle
    syms x
    value_cos = solve(a^2 == b^2 + c^2 - 2*b*c*x, x);

    angle = acos(double(value_cos));

end 


function module = modulusVector(X)

    module = sqrt(X(1)^2 + X(2)^2 + X(3)^2);

end


%% Problem 6

% The trajectories at this problem simply approximate the joint 

