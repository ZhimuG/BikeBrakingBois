function F_F_max = TheoreticalMaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM)

F_F2_max = MaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2);
F_F1_NO = MaximumGroundFrictionNoseOver(M, theta, d_A1_COM);
F_F1_Smax = MaximumGroundFriction(mu_s,d_C2_COM,M,theta,d_C1_C2);
F_F1_max = min(F_F1_NO, F_F1_Smax);

F_F_max = F_F2_max + F_F1_max;
end