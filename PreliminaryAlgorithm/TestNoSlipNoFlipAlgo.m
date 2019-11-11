function TestNoSlipNoFlipAlgo()
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

% Unit tests for the NoSlipNoFlipAlgo
%% TEST 1: Sample calculation from Prototype 1 
p_i_max = 800; % (out of 1024)
p_i = 500; %[V] % INPUT
mu_s = 0.4; %[] % Similar to tire rubber on grass (underestimated for normal cycling conditions)
d_C1_COM = [0.5,1,0]; %[m]
M = 80; %[kg]
theta = 0.1; %[]
d_C1_C2 = 1.2;
SB1 = 0; %[]
R = 0.4; %[m]
r = 0.1; %[m]
I_A2 = 0.2^2*2; %[kg*m^2]
d_C2_COM = [0.7, 1, 0]; %[m]
SB2 = 0; %[]
I_A1 = 0.2^2*2; %[kg*m^2]
d_A1_COM = [0.3,0.6,0]; %[m]

F_F_max = TheoreticalMaximumGroundFriction(mu_s,d_C1_COM,M,0,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM);
[F_b1_out,F_b2_out] = RunNoSlipNoFlipAlgo(F_F_max,p_i_max,p_i,mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM);

tol = 3;
checkEqual(413,F_b2_out,tol)
checkEqual(363.9,F_b1_out,tol)
end

