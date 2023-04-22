clear;close all;clc;
%% L-E dynamic formulation - 3DOF RRR manipulator
syms M1 M2 M3;
syms x1 x1d x1dd x2 x2d x2dd x3 x3d x3dd I1 I2 I3 omega;
syms L1 L2 L3;
syms u1 u2 u3;
syms g

% Position
p1x = 0;
p1y = 0;
p2x = p1x+L2*cos(q1+q2);
p2y = p1y+L2*sin(q1+q2);
p3x = p2x+L3*cos(q1+q2+q3);
p3y = p2y+L3*sin(q1+q2+q3);


% Velocity
v1x = 0;
v1y = 0;
v2x = v1x-L2*sin(q1)*(q1d);
v2y = v1y+L2*cos(q1)*(q1d);
v3x = v2x - L3*sin(q1+q2)*(q1d+q2d);
v3y = v2y + L3*cos(q1+q2)*(q1d+q2d);

% Kinetic energy 
KE = 0.5*M1*( v1x^2 + v1y^2)+0.5*I1 *omega^2 + 0.5*M2*( v2x^2 + v2y^2)+0.5*I2*omega^2 + 0.5*M3*( v3x^2 + v3y^2)+0.5*omega^2*I3;
KE = simplify(KE);

% Potential energy
PE = M1*g*p1y + M2*g*p2y + M3*g*p3y;
PE = simplify(PE);

Px1 = u1;
Px2 = u2;
Px3 = u3;

% dk/dx1d
pKEpx1d = diff(KE,q1d);
% d/dt (dk/dx1d)
ddtpKEpx1d = diff(pKEpx1d,q1)*q1d+ ...
             diff(pKEpx1d,q1d)*q1dd+ ...
             diff(pKEpx1d,q2)*q2d + ...
             diff(pKEpx1d,q2d)*q2dd + ...
             diff(pKEpx1d,q3)*q3d + ...
             diff(pKEpx1d,q3d)*q3dd;
% KE derivative wrt q1
pKEpx1 = diff(KE,q1);
% P derivative wrt q1 
pPEpx1 = diff(PE,q1);

% PE derivative wrt q2 dot, q2
pKEpx2d = diff(KE,q2d);
ddtpKEpx2d = diff(pKEpx2d,q1)*q1d+ ...
             diff(pKEpx2d,q1d)*q1dd+ ...
             diff(pKEpx2d,q2)*q2d + ...
             diff(pKEpx2d,q2d)*q2dd + ...
             diff(pKEpx2d,q3)*q3d + ...
             diff(pKEpx2d,q3d)*q3dd;
pKEpx2 = diff(KE,q2);
pPEpx2 = diff(PE,q2);

%PE derivative wrt q3 dot, q3
pKEpx3d = diff(KE,q3d);
ddtpKEpx3d = diff(pKEpx3d,q1)*q1d+ ...
             diff(pKEpx3d,q1d)*q1dd+ ...
             diff(pKEpx3d,q2)*q2d + ...
             diff(pKEpx3d,q2d)*q2dd + ...
             diff(pKEpx3d,q3)*q3d + ...
             diff(pKEpx3d,q3d)*q3dd;
pKEpx3 = diff(KE,q3);
pPEpx3 = diff(PE,q3);


out_q1 = simplify( ddtpKEpx1d - pKEpx1 + pPEpx1 - Px1);
out_q2 = simplify( ddtpKEpx2d - pKEpx2 + pPEpx2 - Px2);
out_q3 = simplify( ddtpKEpx3d - pKEpx3 + pPEpx3 - Px3);


% Dmatrix：tau derivative wrt q1 dotdot
D11 = diff(out_q1,q1dd);
D21 = diff(out_q2,q1dd);
D31 = diff(out_q3,q1dd);

% D：tau derivative wrt q2 dotdot 
D12 = diff(out_q1,q2dd);
D22 = diff(out_q2,q2dd);
D32 = diff(out_q3,q2dd);

% D：tau derivative wrt q3 dotdot
D13 = diff(out_q1,q3dd);
D23 = diff(out_q2,q3dd);
D33 = diff(out_q3,q3dd);

D = [D11,D12,D13;
     D21,D22,D23;
     D31,D32,D33];
% C：tau derivative wrt q1 dot
C11 = diff(out_q1,q1d);
C21 = diff(out_q2,q1d);
C31 = diff(out_q3,q1d);

% C：tau derivative wrt q2 dot
C12 = diff(out_q1,q2d);
C22 = diff(out_q2,q2d);
C32 = diff(out_q3,q2d);

% C：tau derivative wrt q3 dot
C13 = diff(out_q1,q3d);
C23 = diff(out_q2,q3d);
C33 = diff(out_q3,q3d);

C = [C11,C12,C13;
     C21,C22,C23;
     C31,C32,C33];

% G matrix, potential energy derivatice wrt q
G11 = diff(PE,q1);
G21 = diff(PE,q2);
G31 = diff(PE,q3);

t1 = q1dd*(L2^2*M2 + L2^2*M3 + L3^2*M3 + 2*L2*L3*M3*cos(q2)) - u1 + M3*g*(L2*cos(q1 + q2) +...
    L3*cos(q1 + q2 + q3)) + L2*M2*g*cos(q1 + q2) + L3*M3*q2dd*(L3 + L2*cos(q2)) - ...
    L2*L3*M3*q2d*sin(q2)*(2*q1d + q2d);
t2 = q1dd*(M3*L3^2 + L2*M3*cos(q2)*L3) - u2 + M3*g*(L2*cos(q1 + q2) + L3*cos(q1 + q2 + q3)) +...
    L3^2*M3*q2dd + L2*M2*g*cos(q1 + q2) + L2*L3*M3*q1d^2*sin(q2);
t3 = L3*M3*g*cos(q1 + q2 + q3) - u3;
