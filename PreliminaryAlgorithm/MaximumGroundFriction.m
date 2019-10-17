function [F_F_max] = MaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2)
%UNTITLED Summary of this function goes here

%   d_C1_COM is a 3D vector of the x, y, z components of the displacement
%   from C1 to COM. 
%   theta is the angle of incline in radians 
%   d_C1_C2 is just the x-component since it is defined to point in the
%   x-direction

global g
F_N = M*g*(d_C1_COM(1)*cos(theta)-d_C1_COM(2)*sin(theta))/d_C1_C2;
F_F_max = mu_s*F_N;
end

