function [F_F_desired] = DesiredGroundFriction(p_i,p_i_max,F_F_max)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   Step 1 of preliminary algo

F_F_desired = min(F_F_max, F_F_max*p_i/p_i_max);

end

