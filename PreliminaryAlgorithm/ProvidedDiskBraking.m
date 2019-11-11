function [F_b_provided] = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F_provided)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
F_b_provided = (F_F_provided*R - I_A2*F_F_desired/(R*M))/r;

end

