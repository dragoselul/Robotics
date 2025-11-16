function [T_COMlist, T_list] = computeNumericTransform(poses)
% computeNumericTransform  Returns COM and joint transforms as 4x4x4 arrays
%
% Inputs:
%   poses : [4x1] or [1x4] vector of joint angles [theta1..theta4]
%
% Outputs:
%   T_COMlist : 4x4x4 array, T_COMlist(:,:,i) is COM transform of link i
%   T_list    : 4x4x4 array, T_list(:,:,i)    is joint transform up to link i-1
%               (here: T_list(:,:,1)=T00, T_list(:,:,2)=T01, T_list(:,:,3)=T02, T_list(:,:,4)=T03)

    % Extract the poses
    theta1 = poses(1);
    theta2 = poses(2);
    theta3 = poses(3);
    theta4 = poses(4);

    % Modified DH offsets (for COMs)
    offset_COM1 = 20;
    offset_COM2 = 30;
    offset_COM3 = 30;
    offset_COM4x = 25;
    offset_COM4y = 15;

    % ---------- COM DH transforms (T_i,COM in local link frames) ----------
    T01_COM = DH(theta1,              50 - offset_COM1, 0,            pi/2);
    T12_COM = DH(theta2 + (pi/2),      0,               93 - offset_COM2, 0);
    T23_COM = DH(theta3,               0,               93 - offset_COM3, 0);
    T34_COM = translationMatrix(50 - offset_COM4x, offset_COM4y, 0) * rotationZ(theta4);

    % ---------- Full DH transforms (joint frames) ----------
    T00 = eye(4);
    T01 = DH(theta1,              50, 0,     pi/2);
    T12 = DH(theta2 + (pi/2),      0, 93,    0);
    T23 = DH(theta3,               0, 93,    0);

    T02 = T01 * T12;
    T03 = T01 * T12 * T23;

    % ---------- COM transforms expressed in base frame ----------
    T02_COM = T01       * T12_COM;
    T03_COM = T01       * T12      * T23_COM;
    T04_COM = T01       * T12      * T23      * T34_COM;

    % ---------- Pack into 4x4x4 numeric arrays (Simulink-friendly) ----------
    % Each slice (:,:,i) is a 4x4 transform

    T_COMlist = zeros(4,4,4);
    T_list    = zeros(4,4,4);

    % COM transforms
    T_COMlist(:,:,1) = T01_COM;
    T_COMlist(:,:,2) = T02_COM;
    T_COMlist(:,:,3) = T03_COM;
    T_COMlist(:,:,4) = T04_COM;

    % Joint/base transforms
    T_list(:,:,1) = T00;
    T_list(:,:,2) = T01;
    T_list(:,:,3) = T02;
    T_list(:,:,4) = T03;
end
