function F_F_desired = DesiredGroundFriction(F_F_max,p_i_max,p_i)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   Step 1 of preliminary algo

F_F_desired = min(F_F_max, F_F_max*p_i/p_i_max);

end

