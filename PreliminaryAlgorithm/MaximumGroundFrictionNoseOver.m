function F_F1_NO = MaximumGroundFrictionNoseOver(M, theta, d_A1_COM)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global g;

% Assume a_z=0
% Assume back tire never leaves the ground (but normal force is zero, therefore friction is zero)
% Assume no acceleration about front axle

% Using hungarian equation:
a_x = g*(sin(theta)*d_A1_COM(2)-cos(theta)*d_A1_COM(1))/d_A1_COM(2);
F_F1_NO = -M*(a_x-g*sin(theta));

end

