function [F_b1_out,F_b2_out] = RunNoSlipNoFlipAlgo(F_F_max,p_i_max,p_i,mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM)
%RUNNOSLIPNOFLIPALGO Returns the desired disk-braking forces on the front
%and back wheels.
%   F_b1_out and F_b2_out are the disk-braking forces of the front and 
%   back wheel, respectively, to be sent to the PWM calibration curve.

% Check to make sure I got all of the x, y, z directions correct. 

global g
g = 9.81; %[m/s^2]

% Step 1 of algorithm:
F_F_desired = DesiredGroundFriction(F_F_max, p_i_max, p_i);
% Step 2:
F_F2_max = MaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2);
% Step 3:
leftover = F_F_desired - (F_F2_max - SB1);
if(leftover <= 0)
    % Step 4:
    F_F2_provided = leftover;
    F_b2_provided = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F2_provided);
    F_b1_out = 0;
    F_b2_out = F_b2_provided;
else
    % Step 5:
    F_F2_provided = F_F2_max - SB1;
    F_b2_provided = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F2_provided);
    F_b2_out = F_b2_provided;
    
    % Step 6:
    d_C1_COM(1) = d_C1_COM(1) + 4; % Adjust this forward
    F_F1_NO = MaximumGroundFrictionNoseOver(M, theta, d_A1_COM);
    
    % Step 7:
    F_F1_Smax = MaximumGroundFriction(mu_s,d_C2_COM,M,theta,d_C1_C2);
    
    % Step 8:
    F_F1_max = min(F_F1_NO,F_F1_Smax);
    leftover2 = leftover - (F_F1_max - SB2);
    if(leftover2 <= 0)
        % Step 9
        F_F1_provided = leftover2;
        F_b1_provided = ProvidedDiskBraking(M, R, r, I_A1, F_F_desired, F_F1_provided);
        F_b1_out = F_b1_provided;
    else
        % Step 10:
        F_F1_provided = F_F1_max - SB2;
        F_b1_provided = ProvidedDiskBraking(M, R, r, I_A1, F_F_desired, F_F1_provided);
        F_b1_out = F_b1_provided;
    end
end
end

