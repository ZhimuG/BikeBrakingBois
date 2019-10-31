% Theoretically calculated maximum safe ground-friction force that can be 
% distributed between the two wheels when the bike is on the verge of
% slipping/flipping on flat ground. This value is purposely underestimated.
%F_F_max = 1000; %[N] 
% Potentiometer value of a "firm clamp" defined to cause F_F_max to be exerted. 
p_i_max = 950; % (out of 1024)
% Input potentiometer value
p_i = 500; %[V] % INPUT
% Reasonable (underestimated) value of the coefficient on static friction
mu_s = 0.4; %[] % Similar to tire rubber on grass (underestimated for normal cycling conditions)
% x,y,z components of distance from C1 to COM
d_C1_COM = [0.5,1,0]; %[m]
% Mass of bike and rider 
M = 80; %[kg]
% Angle of incline % INPUT
theta = 0.1; %[]
% Distance between C1 and C2 (in x-direction by construction)
d_C1_C2 = 1.12;
% Safety Buffer 1 (to be determined via tuning)
SB1 = 0; %[]
% Radius of the wheel (assumed to be the same on each wheel)
R = 0.66/2; %[m]
% Radius of the disk brake (assumed to be the same on each wheel)
r = 0.16/2; %[m]
% Moment of inertia of the back wheel about A2 (to be determined via
% experiment)
I_A2 = 0.2^2*2; %[kg*m^2]
% x,y,z components of distance from C2 to COM
d_C2_COM = [0.7, 1, 0]; %[m]
% Safety Buffer 2 (to be determined via tuning)
SB2 = 0; %[]
% Moment of inertia of the front wheel about A2 (to be determined via
% experiment)
I_A1 = 0.2^2*2; %[kg*m^2]
% x,y,z components of distance from A1 to COM. Shorter than d_C1_COM as
% we've purposely assumed the center of mass is futher forward since this
% is only used for the flipping condition
d_A1_COM = [0.3,0.6,0]; %[m]

F_F_max = TheoreticalMaximumGroundFriction(mu_s,d_C1_COM,M,0,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM);
[F_b1_out,F_b2_out] = RunNoSlipNoFlipAlgo(F_F_max,p_i_max,p_i,mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM)
% Return F_b1_out, F_b2_out to control system

%% Range over theta and p_i:
num_theta_range = 100;
num_p_i_range = 200;

theta_range = linspace(-pi/8,pi/8,num_theta_range);
p_i_range = linspace(0, 1024, num_p_i_range);
[THETA,P_I] = meshgrid(theta_range, p_i_range);

F_B1_OUT = zeros(size(THETA));
F_B2_OUT = zeros(size(THETA));
% vectorize this maybe
tic
for i=1:num_theta_range
    for j=1:num_p_i_range
        [F_B1_OUT(j,i),F_B2_OUT(j,i)] = RunNoSlipNoFlipAlgo(F_F_max,p_i_max,p_i_range(j),mu_s,d_C1_COM,M,theta_range(i),d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM);
    end
end
toc
figure(1)
mesh(THETA, P_I, F_B1_OUT);
fs = 16;
xlabel("\theta (radians)","fontsize",fs)
ylabel("p_i","fontsize",fs)
zlabel("F_{b_1} (N)","fontsize",fs)
figure(2)
mesh(THETA, P_I, F_B2_OUT);
xlabel("\theta (radians)","fontsize",fs)
ylabel("p_i","fontsize",fs)
zlabel("F_{b_2} (N)","fontsize",fs)


