function S = skew(v)
    % Computes the skew-symmetric matrix of a 3D vector
    % Input:  v = [vx; vy; vz]
    % Output: S = [v]_x

    S = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0 ];
end