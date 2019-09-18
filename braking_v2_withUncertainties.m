%% Constants
g = 9.81; %[m/s^2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %% Example 1: Using Majd's numbers with tolerances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Inputs for test case from Majd
m_b = 30; %[kg] mass of bike
m_p = 70; %[kg] mass of person
M = m_b + m_p; %[kg] total mass
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
d_COMc2_y = 1.1;  %[m] y-component distance between COM and C2 (in negative y-direction)
d_COMc2_x = 0.46;  %[m] x-component distance between COM and C2 (in negative x-direction)
d_COMCOP_y = 0.2; %[m] y-component distance between COM and COP (in positive? y-direction)
I_COM = 1;      %[kg*m^2] moment of inertia about the COM

%% Example 1a: Minimum stopping distance when using only back brake with error propagation
brakeCase = 2; % Meaning only the back brake is used
% Use Majd's numbers
knownVars =   [ M,mu_s, theta,v_COM_0,d_COMc1_y,d_COMc1_x,d_COMc2_y,d_COMc2_x,d_COMCOP_y,I_COM]'; 
uncertainty = [10, 0.1,pi/180,    0.1,      0.1,      0.1,      0.1,      0.1,         0,    0]';
brakeWithErrorAnon = @(knownVars) brakeWithError(knownVars, brakeCase);
[x_LB,x_UB,f_LB,f_MID,f_UB,minus_percent,plus_percent] = worstcase(brakeWithErrorAnon,knownVars,uncertainty)
%% Example 1b: Minimum stopping distance when using only front brake with error propagation
brakeCase = 1; % Only front brake is used
brakeWithErrorAnon = @(knownVars) brakeWithError(knownVars, brakeCase);
[x_LB,x_UB,f_LB,f_MID,f_UB,minus_percent,plus_percent] = worstcase(brakeWithErrorAnon,knownVars,uncertainty)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %% Example 2: Using our numbers for Test Case 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Inputs
%knownVars =  [  M,mu_s, theta,v_COM_0,d_COMc1_y,d_COMc1_x,d_COMc2_y,d_COMc2_x,d_COMCOP_y,I_COM]';
knownVars =   [100, 0.8,     0,      8,      1.1,     0.54,      1.1,     0.46,       0.1,    1]'; 
uncertainty = [ 10, 0.1,pi/180,    0.1,      0.1,      0.1,      0.1,      0.1,         0,    0]';
%% Example 2a: Minimum stopping distance when using only back brake with error propagation
brakeCase = 2; % Meaning only the back brake is used
brakeWithErrorAnon = @(knownVars) brakeWithError(knownVars, brakeCase);
[x_LB,x_UB,f_LB,f_MID,f_UB,minus_percent,plus_percent] = worstcase(brakeWithErrorAnon,knownVars,uncertainty)
%% Example 2b: Minimum stopping distance when using only front brake with error propagation
tic
brakeCase = 1;
brakeWithErrorAnon = @(knownVars) brakeWithError(knownVars, brakeCase);
[x_LB,x_UB,f_LB,f_MID,f_UB,minus_percent,plus_percent] = worstcase(brakeWithErrorAnon,knownVars,uncertainty)
toc