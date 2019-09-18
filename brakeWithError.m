function d_stop = brakeWithError(knownVars, brakeCase)
g = 9.81;

M = knownVars(1); mu_s = knownVars(2); theta = knownVars(3); v_COM_0 = knownVars(4); d_COMc1_y = knownVars(5); 
d_COMc1_x = knownVars(6); d_COMc2_y = knownVars(7); d_COMc2_x = knownVars(8); d_COMCOP_y = knownVars(9); I_COM = knownVars(10);

% variable order: 
%          [      F_N1,      F_N2,     F_FA2,      F_F2,       F_F1,        F_A, a_y, a_x, alpha]
sumF_y =   [         1,         1,         0,         0,          0,          0,  -M,   0,     0];
sumF_x =   [         0,         0,         1,        -1,         -1,         -1,   0,  -M,     0];
sumM_COM = [ d_COMc1_x,-d_COMc2_x, d_COMc2_y,-d_COMc2_y, -d_COMc1_y, d_COMCOP_y,   0,   0, I_COM]; 

A = [sumF_y; sumF_x; sumM_COM];
b = [M*g*cos(theta); -M*g*sin(theta); 0]; 

% General assumptions
A(:,7) = 0; % A1: Assume there is no acceleration in the y-direction
A(:,3) = 0; % A2: Assume that while braking, no forward force is applied to the back wheel (via pedalling)
A(:,6) = 0; % A3: Assume that there is no air resistance for this model
A(:,9) = 0; % A4: Assume there is no angular acceleration about the center of mass

switch brakeCase
    case 1 %front brake only
        % Front brake assumptions
        A(:,4) = 0; % A7: Assume friction on the back wheel is negligable when only front-braking
        A(:,2) = 0; % A8: Assume that we are operating at the point where the bike is just about to flip (maximum breaking)
    case 2
        % Back brake assumptions
        A(:,5) = 0; % A5: Assume friction on the front wheel is negligable when only back-braking
        A(:,2) = A(:,2)+ mu_s*A(:,4); % A6: Assume we have the maximum value of static friction between the back wheel and pavement (no slipping)
        A(:,4) = 0; % A6
end
 
unknownVars = linsolve(A,b);
F_F1 = unknownVars(5);
F_N1 = unknownVars(1);
F_N2 = unknownVars(2);
a_x = unknownVars(8);
v_COM_f = 0; % Final velocity of the COM
if(F_F1 > mu_s*F_N1) % Check that F_F1 < mu_s 
    disp("WARNING: slipping has likely occcured")
end
d_stop = (v_COM_f^2 - v_COM_0^2)/(2*a_x);

end