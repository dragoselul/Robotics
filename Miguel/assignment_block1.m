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


%% Problem 1 

T01 = DH(0, 50, 0, pi/2);
T12 = DH(pi/2, 0, 93, 0);
T23 = DH(0, 0, 93, 0);
T34 = DH(0, 0, 50, 0);
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 * T34;

% T05 follows a different framework than DH as you cannot attain this
% orientation + position with DH

T35 = translationMatrix(35, 45, 0);
T05 = T03 * T35;

%% Problem 2

% First calculate the joint 3 position (frame 3) 
% From frame 3, both a rotation and transformation are applied to attain
% x04 and o04

% Requirements

% Theta1 = frame 0
% Theta2 = frame 1
% Theta3 = frame 2
% Theta4 = frame 3
% Link1 = 50mm
% Link2 = 93mm
% Link3 = 93mm
% Link4 = 50mm

% Variables

syms theta1 theta2 theta3 theta4

% Define full DH transforms

T01 = DH(theta1, 50, 0, sym(pi/2));
T12 = DH(theta2 + sym(pi/2), 0, 93, 0);
T23 = DH(theta3, 0, 93, 0);
T34 = DH(theta4, 0, 50, 0);
T03 = simplify(T01 * T12 * T23);
T04 = simplify(T01 * T12 * T23 * T34);

function [angles] = IK03(O)
    

    % Between frame 0 and 3 using pure trigonometry with our robot
    
    % where X = vector orientation of x axis in frame 03 3x1
    % where O = point of the center of the coord system in frame 03 3x1

    % Output in DH system ---> needs translation to motor

    x = O(1);
    y = O(2);
    z = O(3);

    theta1 = atan2(y, x); % theta1 sym theta1_ double

    % Following the lecture notes we compute r

    r = hypot(x, y);

    s = z - 50; 

    psi = atan(s/r);

    beta = cosTheorem(hypot(r, s), 93, 93); % same elbow up or down

    phi = atan2(93*sin(pi- beta), 93*cos(pi - beta) + 93);

    theta2_down = -pi/2 + psi - phi;

    theta2_up = -pi/2 + psi + phi;

    % Now theta3 

    theta3_down = pi - beta;

    theta3_up = -theta3_down;


    angles = [theta1, theta2_up, theta3_up; 
              theta1, theta2_down, theta3_down];

end


function point03 = translationPoint34(X, O)


    % function translates point 4 into point 3

    x = O(1);
    y = O(2);

    vector_x = X(1);
    vector_y = X(2);
    vector_z = X(3);

    % Check bounds as we cannot go off plane with this robot config

    if atan2(y, x) ~= atan2(vector_y, vector_x)
        error("Pose not acheivable, vector orientation out of reachable space with given point")
    end 

    % Travel backwards by constant k (modulus scaling)

    k = 50/sqrt(vector_x^2 + vector_y^2 + vector_z^2);

    point03 = O - X * k;

    
end


function transformation = Check3(T, theta_1, theta_2, theta_3)

    syms theta1 theta2 theta3

    transformation = vpa(subs(T, [theta1, theta2, theta3], [theta_1, theta_2 theta_3]), 4);

end

function transformation = Check4(T, theta_1, theta_2, theta_3, theta_4)

    syms theta1 theta2 theta3 theta4

    transformation = vpa(subs(T, [theta1, theta2, theta3, theta4], [theta_1, theta_2 theta_3, theta_4]), 4);

end

function angles = IK4(X_des, O_des, T03sym)
    % X_des : desired end-effector x-axis (3x1) in {0}
    % O_des : desired end-effector origin o_4 (3x1) in {0}
    % T03sym: symbolic T_03 (with theta1,theta2,theta3)

    % the vector orientation defines theta1. Imagine a ball with the end
    % effector spiking all around and pointing out normal to its surface.
    % This is the achievable 3D space + orientation. 
    % Vector (x,y) * k (constant) = point (x, y) otherwise throw error

    L4 = 50;   % link-4 length along x_4
    Xhat = X_des / norm(X_des);            % normalize desired x_4

    % 1) Wrist center
    o3 = O_des - L4 * Xhat;

    % 2) Solve the 2R IK for {0}->o3 (both branches)
    sols03 = IK03(o3);                     % [theta1 theta2 theta3] (2x3)

    % Choose a branch either elbow up (row1) or down (row2)
    t1 = sols03(1,1);  t2 = sols03(1,2);  t3 = sols03(1,3);

    % 3) Build numeric R03 for that branch
    T03 = double(vpa(subs(T03sym, ...
                 {'theta1','theta2','theta3'}, {t1,t2,t3})));
    R03 = T03(1:3,1:3);

    % 4) Feasibility: x4 must be ⟂ z3
    z3 = R03(:,3);
    if abs(dot(Xhat, z3)) > 1e-10
        error('Pose not achievable: desired x4 not perpendicular to z3 at that wrist position.');
    end

    % 5) θ4 from projections on x3,y3
    x3 = R03(:,1);  y3 = R03(:,2);
    c4 = dot(Xhat, x3);
    s4 = dot(Xhat, y3);
    t4 = atan2(s4, c4);

    angles = [t1, t2, t3, t4];
end



sols = IK4([1; 0; 0], [182; 0; 125], T03)
Check4(T04, sols(1, 1), sols(1, 2), sols(1, 3), sols(1, 4));



%% Problem 3 

% The point is defined by a formula that works interestingly, as sweeping
% by the psi angles maps the points on the frame 0 reference frame
% opposite in the y axis looking from the pic perspective. Looking from
% behind the pic, all good

% Anyways, this always maps perfectly as theres no rule as to how you sweep
% since you will be doing 360º

clc

% Variables

syms theta1 theta2 theta3 theta4

% Define full DH transforms

T01 = DH(theta1, 50, 0, sym(pi/2));
T12 = DH(theta2 + sym(pi/2), 0, 93, 0);
T23 = DH(theta3, 0, 93, 0);
T34 = DH(theta4, 0, 50, 0);
T03 = simplify(T01 * T12 * T23);
T04 = simplify(T01 * T12 * T23 * T34);

% Data

p0c = [150, 0, 120]';

% First and last pose are the same so in total 36 poses

psi = 0:2*pi/36:pi*2; 

R = 32;

% Now use of the previous functions and computing the x axis (stylus tip)
% This vector is constrained by theta1 in x and y coordinates

x = p0c(1);

function p_end = CircleDrawer(p_center, R, angles)

    % Angles = range of angles in radians to sweep by the circle
    % R = radius
    % p_center = center of the circle

    p_end = [];
    
    for i = 1:length(angles)

        p_end(:, end + 1) = p_center + R * [0; cos(angles(i)); sin(angles(i))]; 

    end

    % ACTIVATE THIS FOR CHECKING

    % plot(p_end(2, :), p_end(3, :))
    % axis equal

end 

p_end = CircleDrawer(p0c, R, psi);


function poses = configCalculator(points, T03)

    % function works on the plane described by the exercise only, further
    % adjustments and tranform is to be done to be valid everwhere in 3D
    
    poses = [];
    
    
    for i = 1:length(points)

        stylus_tip_vector = [points(1, i); points(2, i); 0]; 

        poses(end+1, :) = IK4(stylus_tip_vector, points(:, i), T03); 

       
    end 

    
end

poses = configCalculator(p_end, T03); 
save("poses.mat", "poses")

function checked = Checker3D(poses, T04)
    point3D = [];
    for i = 1:length(poses)
        trans = Check4(T04, poses(i, 1), poses(i, 2), poses(i, 3), poses(i, 4));
        point3D(:, end+1) = trans(1:3, 4);
    end

    plot3(point3D(1,:), point3D(2,:), point3D(3,:), '-o', ...
      'LineWidth',2, 'MarkerFaceColor','r')
    grid on; axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D points (columns = points)')

end 


% ACTIVATE FOR CHECKING!!!!

% visualizer.m also does it well

% Checker3D(poses, T04); 


%% Problem 4

% The jacobian calculated from the joint points starting o0 until on-1 so
% essentially o3, distances to o4 the stylus tip (end point)

clc


% Essentially the jacobian links the linear and angular velocities of the
% stylus tip to the velocities of the joint angles (its a diff eq)

function jacobian = Jvw(rotation_axis, end_effector, origin)

    % function returns Jv1 Jw1 nx1 vector for a column of the Jacobian in
    % terms of theta1 - theta4
    % ONLY VALID FOR ROTATIONAL JOINTS

    syms theta1 theta2 theta3 theta4

    Jv = skew(rotation_axis) * (end_effector - origin);
    Jw = rotation_axis;

    jacobian = [Jv' , Jw']';
    
end 

function full_jacobian = computeJacobian()

    % In general form in terms of theta1 - theta4

    % Variables

    syms theta1 theta2 theta3 theta4
    
    % Define full DH transforms
    
    T01 = DH(theta1, 50, 0, sym(pi/2));
    T12 = DH(theta2 + sym(pi/2), 0, 93, 0);
    T23 = DH(theta3, 0, 93, 0);
    T34 = DH(theta4, 0, 50, 0);
    T02 = simplify(T01 * T12);
    T03 = simplify(T01 * T12 * T23);
    T04 = simplify(T01 * T12 * T23 * T34);

    o0stylus = T04(1:3, 4);
    o03 = T03(1:3, 4);
    o02 = T02(1:3, 4);
    o01 = T01(1:3, 4);
    o00 = [0; 0; 0];

    o = [o00, o01, o02, o03];

    z00 = [0; 0; 1];
    z01 = T01(1:3, 3);
    z02 = T02(1:3, 3);
    z03 = T03(1:3, 3);

    z = [z00, z01, z02, z03];


    % Loop through to get the Jacobian

    for i = 1:4

        full_jacobian(:, i) = Jvw(z(:, i), o0stylus, o(:, i));

    end

end


% Symbolic Jacobian expression

jacobian_sym = computeJacobian(); 

% Save the symbolic Jacobian expression for exercise 8

save('jacobian_sym_circle', 'jacobian_sym')

load('poses.mat')

pose_index = [1, 36/4 * 1 + 1, 36/4 * 2 + 1, 36/4 * 3 + 1];

jacobian_psi_0 = numericJacobian(jacobian_sym, poses(pose_index(1), :));
jacobian_psi_90 = numericJacobian(jacobian_sym, poses(pose_index(2), :));
jacobian_psi_180 = numericJacobian(jacobian_sym, poses(pose_index(3), :));
jacobian_psi_270 = numericJacobian(jacobian_sym, poses(pose_index(4), :));

save("pose_indexes", "pose_index")
save("jacobian_psi_90", "jacobian_psi_90")
save("jacobian_psi_180", "jacobian_psi_180")
save("jacobian_psi_270", "jacobian_psi_270")
save("jacobian_psi_0", "jacobian_psi_0")

%% Exercise 5

 % Right so the angular velocities at the joints are injected with the
 % jacobian to get the stylus (end-effector) linear velocities vx, vy, vz
 % and angular velocities of the stylus wx, wy, wz

 % So first, extract the actual components of the Jacobian that we want to
 % work with [vx vy vz wz]

 load("jacobian_psi_90.mat");

 Jv = jacobian_psi_90(1:3, :);
 Jw = jacobian_psi_90(6, :);

 Jacobian = [Jv; Jw];

 % Determinant check to see if deficient for 4DOF

if det(Jacobian) < 1e-2
    warning("Deficient square jacobian, continue with pseudoinverse")
    disp(cond(Jacobian))
end

% The circle is constrained within a yz plane fixed in x, so it cannot
% rotate around the z axis. This makes sense for wz = 0

vx = 0;
vy = -3;
vz = 0;
wz = 0;

solution = [vx; vy; vz];

wz = 10;

solution = [vx; vy; vz];

solution_under = [vx; vy; vz; 0; 0; wz];
% Solve the system using the pseudoinverse as the rank is deficient, solve
% for the linear velocity components, if the jacobian is computed
% successfully, I should see wz = 0
% System is overactuated (3 solutions 4 joints (DOFs))

pseudoinverse_J = Jv'*inv(Jv*Jv');

angular_rates = pseudoinverse_J*solution;
pseudoinverse_J_under = inv(jacobian_psi_90' * jacobian_psi_90)* jacobian_psi_90';
angular_rates_under = pseudoinverse_J_under*solution_under;


% ChatGPT interpretation of the wz = 0 condition

J  = numericJacobian(jacobian, q.');   % 6xn
Jv = J(1:3,:);                         % 3xn
Jw = J(4:6,:);                         % 3xn
Jwz = Jw(3,:);                         % 1xn

v_des = [0; -3; 0];

Jvp = pinv(Jv);
qd_prim = Jvp * v_des;

Nv  = eye(size(J,2)) - Jvp*Jv;         % null space of translation
M   = Jwz * Nv;                        % 1xn
% damped scalar LS for robustness:
lambda = 1e-3;
corr = (M'/(M*M' + lambda^2)) * (0 - Jwz*qd_prim);

qd = qd_prim + Nv * corr; 

% Explanation:
% Basically our theta1 (joint1) is the only element of the robot that can
% produce a motion vy and wz at the given point. The primary task will be
% to get the vy = -3 and the second part of the formula leverages the null
% space of the jacobian in order not to change the end effector velocities
% but to optimise the robot's posture. This only has an effect for
% non-defective jacobians where I have enough joints to achieve both the
% primary and secondary tasks. If the secondary task can travel in the
% needed direction in this case wz it would have had an effect. 

% For this case it does not, as Jwz + NullP matrix = 0. No authority to
% change wz without changing the original angular positions (which we dont want as this would lock us out the circle)

