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