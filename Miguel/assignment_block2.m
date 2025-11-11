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

% The trajectories at this problem simply approximates the joint angles,
% rates and accelerations as a function of time by computing a series of
% coefficients

function matrix = coeffMatrix(n, tA, tB)

    % where n is the number of columns of the matrix
    % matrix ALWAYS has 6 rows (qa, qdot_a, qddot_a, qb, qdot_b, qddot_b)
    % tA is the initial boundary condition at t0 
    % tB is the final boundary condition at tfinal

    % this considers the coeffs to be 1 for the angle approx polynomial and
    % cascaded down

    row14 = [linspace(1, 1, n - 1), 1];
    row25 = [polyder(row14), 0];
    row36 = [polyder(polyder(row14)), 0,0];

    row1 = computeNumVect(row14, tA);
    row2 = computeNumVect(row25, tA); 
    row3 = computeNumVect(row36, tA);
    row4 = computeNumVect(row14, tB);
    row5 = computeNumVect(row25, tB);
    row6 = computeNumVect(row36, tB);

    matrix = [row1; row2; row3; row4; row5; row6];

end


function vector = computeNumVect(X, t)

    % where X is a vector (row)
    % functions works out its polinomial evaluation like:
    % [a, b, c] = at^2, bt, c
    % t is where the vector is evaluated

    % remove 0's at the end of the vector

    % add 0 counter
    counter = 0;
    while X(end) == 0
        X(end) = [];
        counter = counter + 1;
    end
    
    n = length(X);

    exponents = n-1:-1:0;

    vector = [];

    for i = 1:n
        
        vector(end+1) = t^exponents(i) * X(i);
        
    end 

    vector = [vector, zeros(1, counter)];

end

% Test

% coeffMatrix(6, 0, 2)


function coeffs = computeCoefficients(n, tA, tB, qa, qdot_a, qddot_a, qb, qdot_b, qddot_b)

    % solutions in 4xn format

    % q = [theta1, theta2, theta3, theta4]'

    solution = [qa', qdot_a, qddot_a, qb', qdot_b, qddot_b];

    % Compute matrix for this solution space between tA and tB

    matrix = coeffMatrix(n, tA, tB);

    q1_coeffs = matrix \ solution(1, :)';
    q2_coeffs = matrix \ solution(2, :)';
    q3_coeffs = matrix \ solution(3, :)';
    q4_coeffs = matrix \ solution(4, :)';

    coeffs = [q1_coeffs'; q2_coeffs'; q3_coeffs'; q4_coeffs'];

end

% ---- LOAD THE POSES, JACOBIANS AND POSITIONS AT 0 90 180 270º ---- 

load("pose_indexes.mat")
load("poses.mat")
load('jacobian_psi_0.mat')
load('jacobian_psi_90.mat')
load('jacobian_psi_180.mat')
load('jacobian_psi_270.mat')

% These are always 0

qddot_a = [0; 0; 0; 0];
qddot_b = [0; 0; 0; 0];

% Compute the circle points

circle_center = [150; 0; 120];
R = 32;
angles = 0:2*pi/36:pi*2;
circle_points = circleDrawer(circle_center, R, angles);

% --- FIRST QUADRANT (t = 0-2s) -----

qa = poses(pose_index(1), :);
qb = poses(pose_index(2), :);
va = [0; 0; 0]; % from exercise conditions
vb = [0; -27; 0]; % from exercise conditions
tA = 0; % from exercise conditions
tB = 2; % from exercise conditions

pa = circle_points(:, pose_index(1)); % point at position A
pb = circle_points(:, pose_index(2)); % point at position B

% Now calc the angular rates qdot_a and qdot_b

JvA = jacobian_psi_0(1:3, :);
JvB = jacobian_psi_90(1:3, :);

% Overactuated arm pseudoinverse

pseudoinverse_J_A = JvA'*inv(JvA*JvA');
pseudoinverse_J_B = JvB'*inv(JvB*JvB');

qdot_a = double(pseudoinverse_J_A*va);
qdot_b = double(pseudoinverse_J_B*vb); 

% Theta1 to Theta4 coefficients

Acoeffs = computeCoefficients(5, tA, tB, qa, qdot_a, qddot_a, qb, qdot_b, qddot_b)

% --- SECOND QUADRANT (t = 2-4s) -----

indexA = pose_index(2);
indexB = pose_index(3);

qa = poses(indexA, :); % pose at point psi 90º
qb = poses(indexB, :); % pose at point psi 180º
va = [0; -27; 0]; % from exercise conditions
vb = [0; 0; -27]; % from exercise conditions
tA = 2; % from exercise conditions
tB = 4; % from exercise conditions

pa = circle_points(:, indexA); % point at position A
pb = circle_points(:, indexB); % point at position B

% Now calc the angular rates qdot_a and qdot_b

JvA = jacobian_psi_90(1:3, :);
JvB = jacobian_psi_180(1:3, :);

% Overactuated arm pseudoinverse

pseudoinverse_J_A = JvA'*inv(JvA*JvA');
pseudoinverse_J_B = JvB'*inv(JvB*JvB');

qdot_a = double(pseudoinverse_J_A*va);
qdot_b = double(pseudoinverse_J_B*vb); 

% Theta1 to Theta4 coefficients

Bcoeffs = computeCoefficients(5, tA, tB, qa, qdot_a, qddot_a, qb, qdot_b, qddot_b)

% --- THIRD QUADRANT (t = 4-6s) -----

indexA = pose_index(3);
indexB = pose_index(4);

qa = poses(indexA, :); % pose at point psi 90º
qb = poses(indexB, :); % pose at point psi 180º
va = [0; 0; -27]; % from exercise conditions
vb = [0; 27; 0]; % from exercise conditions
tA = 4; % from exercise conditions
tB = 6; % from exercise conditions

pa = circle_points(:, indexA); % point at position A
pb = circle_points(:, indexB); % point at position B

% Now calc the angular rates qdot_a and qdot_b

JvA = jacobian_psi_90(1:3, :);
JvB = jacobian_psi_180(1:3, :);

% Overactuated arm pseudoinverse

pseudoinverse_J_A = JvA'*inv(JvA*JvA');
pseudoinverse_J_B = JvB'*inv(JvB*JvB');

qdot_a = double(pseudoinverse_J_A*va);
qdot_b = double(pseudoinverse_J_B*vb); 

% Theta1 to Theta4 coefficients

Ccoeffs = computeCoefficients(5, tA, tB, qa, qdot_a, qddot_a, qb, qdot_b, qddot_b)


% --- FOURTH QUADRANT (t = 6-8s) -----

indexA = pose_index(4);
indexB = pose_index(1);

qa = poses(indexA, :); % pose at point psi 90º
qb = poses(indexB, :); % pose at point psi 180º
va = [0; 27; 0]; % from exercise conditions
vb = [0; 0; 0]; % from exercise conditions
tA = 6; % from exercise conditions
tB = 8; % from exercise conditions

pa = circle_points(:, indexA); % point at position A
pb = circle_points(:, indexB); % point at position B

% Now calc the angular rates qdot_a and qdot_b

JvA = jacobian_psi_90(1:3, :);
JvB = jacobian_psi_180(1:3, :);

% Overactuated arm pseudoinverse

pseudoinverse_J_A = JvA'*inv(JvA*JvA');
pseudoinverse_J_B = JvB'*inv(JvB*JvB');

qdot_a = double(pseudoinverse_J_A*va);
qdot_b = double(pseudoinverse_J_B*vb); 

% Theta1 to Theta4 coefficients

Dcoeffs = computeCoefficients(5, tA, tB, qa, qdot_a, qddot_a, qb, qdot_b, qddot_b)


%% Problem 7

% RUN EXERCISE 6 BEFORE DOING 7!!!!!! DO NOT CLEAN THE OUTPUT

% Essentially a time continuous function that takes my theta1(t) to theta4(t) and gives me a robot path. This can be achieved running the 
% homogenous transformation matrix discretly from frame 0 until frame 4

% Take these from problem 2

% Variables

syms theta1 theta2 theta3 theta4

% Define full DH transforms

T01 = DH(theta1, 50, 0, sym(pi/2));
T12 = DH(theta2 + sym(pi/2), 0, 93, 0);
T23 = DH(theta3, 0, 93, 0);
T34 = DH(theta4, 0, 50, 0);
T03 = simplify(T01 * T12 * T23);
T04 = simplify(T01 * T12 * T23 * T34);

% First, extract the function fx(theta1(t), theta2(t), theta3(t),
% theta4(t)), fy and fz respectively for the point. This is essentially a
% function of a function 

point_functionXYZ = extractXYZ(T04); % point [x, y, z]' vector

% Create a function that can loop through the point function between 2 time
% intervals and a defined set of coefficients

global number_points

number_points = 40;

function points = computePlot(tA, tB, coeff, point_functionXYZ)

    syms theta1 theta2 theta3 theta4

    global number_points

    % tA - initial time
    % tB - final time 
    % coeff - coefficients matrix either A, B, C or D (gives coeffs to all the theta1(t) - theta4(t) expressions)

    % Compute the numerical value of the vector components evaluated at a
    % time t and then sum the elements to get the vector, numerical value

    % Use a loop for that, then add all of them into a list of points [3, n]

    % Define the number of time intervals t (number of points) 

    points = zeros(3, number_points);

    index = 1;

    for t = linspace(tA, tB, number_points) % 40 points per interval
      
        theta1_t = sum(computeNumVect(coeff(1, :), t));
        theta2_t = sum(computeNumVect(coeff(2, :), t));
        theta3_t = sum(computeNumVect(coeff(3, :), t));
        theta4_t = sum(computeNumVect(coeff(4, :), t));

        % With the fed xyz function, subs the theta values by the computed
        % thetas at that time interval for the X, Y and Z coordinates

        points(:, index) = subs(point_functionXYZ, [theta1, theta2, theta3, theta4], [theta1_t, theta2_t, theta3_t, theta4_t]);

        index = index + 1;
             
    end

end 

% Compute the trajectories for the circle 

circle_trajectory = zeros(3, number_points * 4);

coeffs_all = {Acoeffs, Bcoeffs, Ccoeffs, Dcoeffs};

indexer = [0];

% where i defines the 4 quadrants of the circle 
time_interval = [0, 2];

for i = 1:4

    % Keep track of the number of points to add to the list in batches of
    % number of points 

    indexer(end+1) = i * number_points; 

    circle_trajectory(:, indexer(i) + 1 : indexer(i + 1)) = computePlot(time_interval(1), time_interval(2), coeffs_all{i}, point_functionXYZ);

    time_interval = time_interval + 2;
end 

% Generate ideal circular path
angles = linspace(0, 2*pi, size(ee_positions, 1));
circle_center = [150; 0; 120];
R = 32;
circle_path = circle_center + R * [zeros(size(angles));
                                   cos(angles);
                                   sin(angles)];
circle_path = circle_path';


% Plot comparison
figure;
plot3(circle_trajectory(1, :), circle_trajectory(2, :), circle_trajectory(3, :), 'LineWidth', 1.8);
hold on;
plot3(circle_path(:,1), circle_path(:,2), circle_path(:,3), 'r--', 'LineWidth', 2);
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('Interpolated Path', 'Ideal Circular Path');
title('End-Effector Trajectory vs. Ideal Circle');
grid on; axis equal;