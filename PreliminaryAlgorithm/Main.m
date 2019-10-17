% Theoretically calculated maximum safe ground-friction force that can be 
% distributed between the two wheels when the bike is on the verge of
% slipping/flipping on flat ground. This value is purposely underestimated.
F_F_max = 1000; %[N] 
% Potentiometer voltage of a "firm clamp" defined to cause F_F_max to be exerted. 
p_i_max = 0.5; %[V]
% Input potentiometer value
p_i = 0.2; %[V] % INPUT
% Reasonable (underestimated) value of the coefficient on static friction
mu_s = 0.3; %[]
% x,y,z components of distance from C1 to COM
d_C1_COM = [0.3,0.2,0.1]; %[m]
% Mass of bike and rider 
M = 100; %[kg]
% Angle of incline % INPUT
theta = 0.1; %[]
% Distance between C1 and C2 (in x-direction by construction)
d_C1_C2 = 0.5;
% Safety Buffer 1 (to be determined via tuning)
SB1 = 0; %[]
% Radius of the wheel (assumed to be the same on each wheel)
R = 0.1; %[m]
% Radius of the disk brake (assumed to be the same on each wheel)
r = 0.03; %[m]
% Moment of inertia of the back wheel about A2 (to be determined via
% experiment)
I_A2 = 0.1; %[kg*m^2]
% x,y,z components of distance from C2 to COM
d_C2_COM = [0.4, 0.2, 0.1]; 
% Safety Buffer 2 (to be determined via tuning)
SB2 = 0;
% Moment of inertia of the front wheel about A2 (to be determined via
% experiment)
I_A1 = 0.1;
% x,y,z components of distance from A1 to COM
d_A1_COM = [0.3,0.1,0.2];

tic
[F_b1_out,F_b2_out] = RunNoSlipNoFlipAlgo(F_F_max,p_i_max,p_i,mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM);
toc
% Return F_b1_out, F_b2_out to control system