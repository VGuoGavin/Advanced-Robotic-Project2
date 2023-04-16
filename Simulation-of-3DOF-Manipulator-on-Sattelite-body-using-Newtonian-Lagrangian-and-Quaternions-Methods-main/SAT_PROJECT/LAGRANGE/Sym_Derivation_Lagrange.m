clc 
clear 
close all
t0=clock;
%% SYMBOLIC VARIABLE
syms t
syms a b c
syms L1 L2 L3
syms Ms m1 m2 m3
syms Ixx Ixy Ixz Iyy Iyz Izz 
syms q1 q2 q3 q4 q5 q6 q7 q8 q9
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 dq8 dq9
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 ddq7 ddq8 ddq9
% q1=Xg, q2=Yg q3=Zg, x4=Sai, x5=Teta, x6=Phi, x7=Teta1, x8=Teta2, x9=Teta2,
R01 = [cos(q4) sin(q4) 0;-sin(q4) cos(q4) 0;0 0 1]; %XYZ-------->x1y1z1
R12 = [cos(q5) 0 -sin(q5);0 1 0;sin(q5) 0 cos(q5)]; %x1y1z1----->x2y2z2
R23 = [1 0 0;0 cos(q6) sin(q6);0 -sin(q6) cos(q6)]; %x2y2z2----->x3y3z3
R34 = [cos(q7) sin(q7) 0;-sin(q7) cos(q7) 0;0 0 1]; %x3y3z3----->x4y4z4
R45 = [cos(q8) 0 -sin(q8);0 1 0;sin(q8) 0 cos(q8)]; %x4y4z4----->x5y5z5
R56 = [cos(q9) 0 -sin(q9);0 1 0;sin(q9) 0 cos(q9)]; %x5y5z5----->x6y6z6
%coordinate jumping
R03 = R23*R12*R01;                                  %XYZ-------->x3y3z3
R35 = R45*R34;                                      %x3y3z3----->x5y5z5
R36 = R56*R45*R34;                                  %x3y3z3----->x6y6z6
R46 = R56*R45;                                      %x4y4z4----->x6y6z6
%% MOIs
I_sat_x3y3z3 = [Ixx -Ixy -Ixz;-Ixy Iyy -Iyz;-Ixz -Iyz Izz];
I_LINK1_x4y4z4 = 1/12*m1*L1^2*[1 0 0;0 1 0;0 0 1/5];
I_LINK2_x5y5z5 = 1/12*m2*L2^2*[1/5 0 0;0 1 0;0 0 1];
I_LINK3_x6y6z6 = 1/12*m3*L3^2*[1/5 0 0;0 1 0;0 0 1];

%% SATELLITE
w_sat_x3y3z3 = R23*R12*R01*[0;0;dq4]+R23*R12*[0;dq5;0]+R23*[dq6;0;0];

vG_sat_XYZ = [dq1;dq2;dq3];
vG_sat_x3y3z3 = R03*vG_sat_XYZ;

HG_sat_x3y3z3  = I_sat_x3y3z3*w_sat_x3y3z3;
T_SAT = 1/2*Ms*(vG_sat_XYZ).' * (vG_sat_XYZ)+...
        1/2*(w_sat_x3y3z3).' * HG_sat_x3y3z3;
    
%% LINK1
rgLINK1_G_x3y3z3 = [0;0;c/2+L1/2];
w_LINK1_x4y4z4 = R34 * w_sat_x3y3z3 + dq7*[0;0;1];

vg_LINK1_x3y3z3 = vG_sat_x3y3z3 + cross(w_sat_x3y3z3,rgLINK1_G_x3y3z3);
vg_LINK1_x4y4z4 = R34*vg_LINK1_x3y3z3;

Hg_LINK1_x4y4z4 = I_LINK1_x4y4z4 * w_LINK1_x4y4z4;

T_LINK1 = 1/2*m1*(vg_LINK1_x3y3z3).' * (vg_LINK1_x3y3z3)+...
          1/2*(w_LINK1_x4y4z4).' * Hg_LINK1_x4y4z4;
%% joint2
r_joint2_x3y3z3 = [0;0;c/2+L1];
v_joint2_x3y3z3 = vG_sat_x3y3z3 + cross(w_sat_x3y3z3,r_joint2_x3y3z3);
v_joint2_x4y4z4 = R34*v_joint2_x3y3z3;
%% LINK2
rgLINK2_G_x5y5z5 = R35*r_joint2_x3y3z3 + [L2/2;0;0];
w_LINK2_x5y5z5 = R45*(w_LINK1_x4y4z4+dq8*[0;1;0]);

vg_LINK2_x5y5z5 = R45 *v_joint2_x4y4z4 + cross(w_LINK2_x5y5z5,[L2/2;0;0]);

Hg_LINK2_x5y5z5 = I_LINK2_x5y5z5 * w_LINK2_x5y5z5;

T_LINK2 = 1/2*m2* (vg_LINK2_x5y5z5).' * (vg_LINK2_x5y5z5)+...
          1/2*(w_LINK2_x5y5z5).' * Hg_LINK2_x5y5z5;
%% joint3
r_joint3_x5y5z5 = R35*r_joint2_x3y3z3 + [L2;0;0];
v_joint3_x5y5z5 = R45*v_joint2_x4y4z4 + cross(w_LINK2_x5y5z5,[L2;0;0]);

%% LINK3
w_LINK3_x6y6z6 = R56*(w_LINK2_x5y5z5 + dq9*[0;1;0]); 

rgLINK3_G_x6y6z6 = R56*(r_joint3_x5y5z5) + [L3/2;0;0];
vg_LINK3_x6y6z6 = R56 * v_joint3_x5y5z5 + cross(w_LINK3_x6y6z6,[L3/2;0;0]);

Hg_LINK3_x6y6z6 = I_LINK3_x6y6z6 * w_LINK3_x6y6z6;

T_LINK3 = 1/2*m3* (vg_LINK3_x6y6z6).' * (vg_LINK3_x6y6z6)+...
          1/2*(w_LINK3_x6y6z6).' * Hg_LINK3_x6y6z6;
    
%% KINETIC AND POTENTIAL ENERGY-->LAGRANZHIAN
T = T_SAT + T_LINK1 + T_LINK2 + T_LINK3;
V = 0;
L = T-V;
E = T+V;
%% Generalized coordinates
q=[q1;q2;q3;q4;q5;q6;q7;q8;q9];
dq=[dq1;dq2;dq3;dq4;dq5;dq6;dq7;dq8;dq9];

%%
dL_dq = jacobian(L,q);
dL_ddq = jacobian(L,dq);

M = jacobian(dL_ddq,dq);
N = jacobian(dL_ddq,q);
A = diff(dL_ddq.',t);
B = N*dq + A - dL_dq.';

%% Linear and Angular Momentum
P_XYZ = Ms*vG_sat_XYZ+m1*R03.'*vg_LINK1_x3y3z3+m2*(R35*R03).'*vg_LINK2_x5y5z5+m3*(R36*R03).'*vg_LINK3_x6y6z6;

rGsat_XYZ = [q1;q2;q3];
Hsat_XYZ = R03.'*HG_sat_x3y3z3 + Ms*cross(rGsat_XYZ,vG_sat_XYZ);

rg_LINK1_XYZ = rGsat_XYZ + R03.'*(rgLINK1_G_x3y3z3);
Hg_LINK1_XYZ = (R34*R03).'*Hg_LINK1_x4y4z4 + m1*cross(rg_LINK1_XYZ,R03.'*vg_LINK1_x3y3z3);

rg_LINK2_XYZ = rGsat_XYZ+(R35*R03).'*rgLINK2_G_x5y5z5;
Hg_LINK2_XYZ = (R35*R03).'*Hg_LINK2_x5y5z5 + m2*cross(rg_LINK2_XYZ,(R35*R03).'*vg_LINK2_x5y5z5);


rg_LINK3_XYZ = rGsat_XYZ+(R36*R03).'*rgLINK3_G_x6y6z6;
Hg_LINK3_XYZ = (R36*R03).'*Hg_LINK3_x6y6z6 + m3*cross(rg_LINK3_XYZ,(R36*R03).'*vg_LINK3_x6y6z6);

H_XYZ = Hsat_XYZ + Hg_LINK1_XYZ + Hg_LINK2_XYZ + Hg_LINK3_XYZ;

%% matlabFunctions
MFunc=matlabFunction(M,'File','Mfunc');
BFunc=matlabFunction(B,'File','generate11\Bfunc');
EFunc=matlabFunction(E,'File','generate11\Efunc');
PFunc=matlabFunction(P_XYZ,'file','generate11\Pfunc');
HFunc=matlabFunction(H_XYZ,'file','generate11\Hfunc');
%% END
t1=clock;
ti=etime(t1,t0)/60;
disp(['Newtonian Symbolic Derivation Simulation Time: ' num2str(ti) '(min)'])
