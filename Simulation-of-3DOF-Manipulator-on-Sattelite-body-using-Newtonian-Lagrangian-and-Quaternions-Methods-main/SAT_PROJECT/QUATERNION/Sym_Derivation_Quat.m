clc
clear all
close all
t0=clock;
%% symbolic parameters
syms t
syms c L1 L2 L3
syms Ms m1 m2 m3 
syms q1 q2 q3 e0 e1 e2 e3 q7 q8 q9
syms dq1 dq2 dq3 de0 de1 de2 de3 dq7 dq8 dq9
syms Ixx Iyy Izz Ixy Ixz Iyz
% q1=Xg, q2=Yg, q3=Zg, e0=cos(alph/2), e1=c1*sin(alph/2),e2=c2*sin(alph/2),e3=c3*sin(alph/2), 
% q7=Teta1, q8=Teta2, q9=Teta2,
%% MOMENT OF INNERTIA
I_sat_x3y3z3 = [Ixx -Ixy -Ixz;-Ixy Iyy -Iyz;-Ixz -Iyz Izz];
I_LINK1_x4y4z4 = 1/12*m1*L1^2*[1 0 0;0 1 0;0 0 1/5];
I_LINK2_x5y5z5 = 1/12*m2*L2^2*[1/5 0 0;0 1 0;0 0 1];
I_LINK3_x6y6z6 = 1/12*m3*L3^2*[1/5 0 0;0 1 0;0 0 1];

%% ROTATION MATRIXS
R03 = [e0^2+e1^2-e2^2-e3^2 2*(e1*e2+e0*e3) 2*(e1*e3-e0*e2);
       2*(e1*e2-e0*e3) e0^2-e1^2+e2^2-e3^2 2*(e2*e3+e0*e1);
       2*(e1*e3+e0*e2) 2*(e2*e3-e0*e1) e0^2-e1^2-e2^2+e3^2];  %XYZ------->x3y3z3
R34 = [cos(q7) sin(q7) 0;-sin(q7) cos(q7) 0;0 0 1]; %x3y3z3----->x4y4z4
R45 = [cos(q8) 0 -sin(q8);0 1 0;sin(q8) 0 cos(q8)]; %x4y4z4----->x5y5z5
R56 = [cos(q9) 0 -sin(q9);0 1 0;sin(q9) 0 cos(q9)]; %x5y5z5----->x6y6z6
%coordinate jumping
R35 = R45*R34;                                      %x3y3z3----->x5y5z5
R36 = R56*R45*R34;                                  %x3y3z3----->x6y6z6
R46 = R56*R45;                                      %x4y4z4----->x6y6z6

W=2*[-e1 e0 e3 -e2;-e2 -e3 e0 e1;-e3 e2 -e1 e0]*[de0;de1;de2;de3];
%% SATELLITE
w_sat_x3y3z3 = W;

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
 
%% KINETIC AND POTENTIAL ENERGY---->LAGRANGIAN
T = T_SAT + T_LINK1 + T_LINK2 + T_LINK3;
V = 0;
E=T+V;
L=T-V;

%% Generalized Coordinates
q=[q1;q2;q3;e0;e1;e2;e3;q7;q8;q9];
dq=[dq1;dq2;dq3;de0;de1;de2;de3;dq7;dq8;dq9];
%% Mass and Bias Matrixs
dL_dq = jacobian(L,q);
dL_ddq = jacobian(L,dq);

M = jacobian(dL_ddq,dq);
N = jacobian(dL_ddq,q);
A = diff(dL_ddq.',t);
B = N*dq + A - dL_dq.';
% There is no need to simplify?
% M=simplify(M);
% B=simplify(B);
%% linear and Angular Momentum
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
MFunc=matlabFunction(M,'File','generate71\MFunc');
BFunc=matlabFunction(B,'File','generate71\BFunc');
PFunc=matlabFunction(P_XYZ,'File','generate71\PFunc');
HFunc=matlabFunction(H_XYZ,'File','generate71\HFunc');
EFunc=matlabFunction(E,'File','generate71\EFunc');

%%
t1=clock;
ti=etime(t1,t0)/60;
disp(['Quaternions Symbolic Derivation Simulation Time: ' num2str(ti) '(min)'])
%% END




