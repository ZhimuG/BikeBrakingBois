%% Inputs
m_b = 30; %[kg] mass of bike
m_p = 70; %[kg] mass of person
mu_s = 0.8; %[] coefficient of static friction
theta = pi/36; %[radians] angle of incline
v_COM_0 = 8.00; %[m/s] initial linear velocity of center of mass

% For Torque about front wheel (C1)
d_c1c2 = 1; %[m] distance between C1 and C2 (in negative x-direction)
d_c1COP_y = 1; %[m] y-component distance between C1 and COP  (in positive y-direction)
d_c1COM = 1; %[m] distance between C1 and COM 

% For Torque about center of mass (COM)
d_COMc1_y = 1.1;  %[m] y-component distance between COM and C1 (in negative y-direction)
d_COMc1_x = 0.54;  %[m] x-component distance between COM and C1 (in positive x-direction)
d_COMCOP_y = 0.2; %[m] y-component distance between COM and COP (in positive? y-direction)
d_COMc2_y = 1.1;  %[m] y-component distance between COM and C2 (in negative y-direction)
d_COMc2_x = 0.46;  %[m] x-component distance between COM and C2 (in negative x-direction)
I_COM = 1;      %[kg*m^2] moment of inertia about the COM
%% Preprocessing
g = 9.81; %[m/s^2] 
M = m_b + m_p;

% variable order: 
%          [      F_N1,      F_N2,     F_FA2,      F_F2,       F_F1,        F_A, a_y, a_x, alpha
sumF_y =   [         1,         1,         0,         0,          0,          0,  -M,   0, 0];
sumF_x =   [         0,         0,         1,        -1,         -1,         -1,   0,  -M, 0];
sumM_COM = [ d_COMc1_x,-d_COMc2_x, d_COMc2_y,-d_COMc2_y, -d_COMc1_y, d_COMCOP_y,   0,   0, I_COM]; 

A = [sumF_y; sumF_x; sumM_COM];
b = [M*g*cos(theta); -M*g*sin(theta); 0]; 

%% Assumptions
A(:,7) = 0; % A1: Assume there is no acceleration in the y-direction
A(:,3) = 0; % A2: Assume that while braking, no forward force is applied to the back wheel (via pedalling)
A(:,6) = 0; % A3: Assume that there is no air resistance for this model
A(:,9) = 0; % A4: Assume there is no angular acceleration about the center of mass

%% Example 1: Minimum stopping distance when using only back brake
Aex1 = A;
Aex1(:,5) = 0; % A5: Assume friction on the front wheel is negligable when only back-braking
Aex1(:,2) = Aex1(:,2)+ mu_s*Aex1(:,4); % A6: Assume we have the maximum value of static friction between the back wheel and pavement (no slipping)
Aex1(:,4) = 0; % A6

ex1 = linsolve(Aex1,b);
F_N1 = ex1(1);
F_N2 = ex1(2);
a_x = ex1(8);
v_COM_f = 0; % Final velocity of the COM
d_stop = (v_COM_f^2 - v_COM_0^2)/(2*a_x);

%% Example 2: Minimum stopping distance when using only the front brake
Aex2 = A;
Aex2(:,5) = 0; % A7: Assume friction on the back wheel is negligable when only front-braking
Aex2(:,2) = 0; % A8: Assume that we are operating at the point where the bike is just about to flip (maximum breaking)

ex2 = linsolve(Aex2,b);
F_F1 = ex2(4);
F_N1 = ex2(1);
a_x = ex2(8);
if(F_F1 > mu_s*F_N1) % Check that F_F1 < mu_s 
    disp("WARNING: slipping has likely occcured")
end
d_stop = (v_COM_f^2 - v_COM_0^2)/(2*a_x);
