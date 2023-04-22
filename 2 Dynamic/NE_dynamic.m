%% Derive Newton-Euler formulation -- 3DOF RRR manipulator

syms l1 l2 l3 m g I w0 dw0 dv0 c1 s1 c2 s2 c3 s3 dq dq1 dq2 dq3 dw Pc F N f n m1 m2 m3 ddq1 ddq2 ddq3
syms q0 dq0 ddq0
R=cell(1,6);w=cell(1,6);dw=cell(1,6);dq=cell(1,6);ddq=cell(1,6);dv=cell(1,6);
f=cell(1,6);n=cell(1,6);P=cell(1,6);Pc=cell(1,6);F=cell(1,6);m=cell(1,6);N=cell(1,6);
I=cell(1,6);

%Initial value
R{1}=[c1 -s1 0;s1 c1 0;0 0 1];R{2}=[c2 0 s2 ;0 1 0;-s2  0 c2 ];R{3}=[c3 0 s3 ;0 1 0;-s3  0 c3 ]; 
R{4} = [1 0 0;0 1 0; 0 0 1];


dv{1}=[0;g;0];
%Coordinate position, offset condition
P{1}=[0;0;0];P{2}=[0;0;0];P{3}=[l1;0;0];P{4}=[l2;0;0];
%the centre of mass of each link
Pc{2}=[0;0;0];Pc{3}=[l1;0;0];Pc{4}=[l2;0;0];
%the mass of each link
m{2}=0;m{3}=m1;m{4}=m2;
I{2}=[0;0;0];I{3}=[0;0;0];
%the base of robot is rotating
w{1}=[0;0;q0];
dw{1}=[0;0;dq0];
%the rotating joints
dq{2}=[0;0;dq0];dq{3}=[0;dq1;0];dq{4}=[0;dq2;0];
ddq{2}=[0;0;ddq0];ddq{3}=[0;ddq1;0];ddq{4}=[0;ddq2;0];
%no force exerted on end-effector
f{5}=[0;0;0];
n{5}=[0;0;0];


%% Outward iteration
for i=1:3
w{i+1}=R{i}.'*w{i}+dq{i+1};
dw{i+1}=R{i}.'*dw{i}+cross(R{i}.'*w{i},dq{i+1})+ddq{i+1};
dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};
F{i+1}=m{i+1}*dvc{i+1};
%Momement of inertia is zero
N{i+1}=[0;0;0];
end


%% Inward iteration
for i=4:-1:2
    f{i}=R{i}*f{i+1}+F{i};
    n{i}=N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end

%% torque
tau3=n{4}(3,1);
tau2=n{3}(3,1);
tau1=n{2}(3,1);



% D matrix, column one：derivative of tau wrt to q1 dotdot
D11 = diff(tau1,ddq1);
D21 = diff(tau2,ddq1);
D31 = diff(tau3,ddq1);

% D matrix, column two：derivative of tau wrt to q2 dotdot
D12 = diff(tau1,ddq2);
D22 = diff(tau2,ddq2);
D32 = diff(tau3,ddq3);

% D matrix, column three：derivative of tau wrt to q3 dotdot
D13 = diff(tau1,ddq3);
D23 = diff(tau2,ddq3);
D33 = diff(tau3,ddq3);

D = [D11,D12,D13;
     D21,D22,D23;
     D31,D32,D33];

% C matrix, column one：derivative of tau wrt to q1 dot
C11 = diff(tau1,dq1);
C21 = diff(tau2,dq1);
C31 = diff(tau3,dq1);

% C matrix, column two：derivative of tau wrt to q2 dot
C12 = diff(tau1,dq2);
C22 = diff(tau2,dq2);
C32 = diff(tau3,dq3);

% C matrix, column three：derivative of tau wrt to q3 dot
C13 = diff(tau1,dq3);
C23 = diff(tau2,dq3);
C33 = diff(tau3,dq3);

C = [C11,C12,C13;
     C21,C22,C23;
     C31,C32,C33];

% G：tau derivative wrt q1 dot
% G11 = diff(tau1,~);
% G21 = diff(tau2,q1);
% G31 = diff(tau3,q1);
% 
% % G：tau derivative wrt q2 dot
% G12 = diff(tau1,q2);
% G22 = diff(tau2,q2);
% G32 = diff(tau3,q3);
% 
% % G：tau derivative wrt q3 dot
% G13 = diff(tau1,q3);
% G23 = diff(tau2,q3);
% G33 = diff(tau3,q3);
% 
% G = [G11,G12,G13;
%      G21,G22,G23;
%      G31,G32,G33];

t1 = l2*m2*s2*s3*(l2*(s3*(s2*(ddq0 + dq0) + c2*dq1*(dq0 + q0)) - c3*(c2*(ddq0 + dq0) -...
    dq1*s2*(dq0 + q0)) + dq2*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0))) - c1*g - l1*(c2*(ddq0 + dq0) - ...
    dq1*s2*(dq0 + q0)) + l2*(dq1 + dq2)*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0)) + dq1*l1*s2*(dq0 + q0)) - ...
    c2*(l1*m2*(l2*(s3*(s2*(ddq0 + dq0) + c2*dq1*(dq0 + q0)) - c3*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + ...
    dq2*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0))) - c1*g - l1*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + l2*(dq1 + ...
    dq2)*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0)) + dq1*l1*s2*(dq0 + q0)) - l1*m1*(c1*g + l1*(c2*(ddq0 + dq0) -...
    dq1*s2*(dq0 + q0)) - dq1*l1*s2*(dq0 + q0)) + c3*l2*m2*(l2*(s3*(s2*(ddq0 + dq0) + c2*dq1*(dq0 + q0)) - ...
    c3*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + dq2*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0))) - c1*g - l1*(c2*(ddq0 + dq0)...
    - dq1*s2*(dq0 + q0)) + l2*(dq1 + dq2)*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0)) + dq1*l1*s2*(dq0 + q0)));

t2 = l1*m1*(c1*g + l1*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) - dq1*l1*s2*(dq0 + q0)) - l1*m2*(l2*(s3*(s2*(ddq0 + dq0) +...
    c2*dq1*(dq0 + q0)) - c3*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + dq2*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0))) - c1*g - ...
    l1*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + l2*(dq1 + dq2)*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0)) + dq1*l1*s2*(dq0 + q0)) - ...
    c3*l2*m2*(l2*(s3*(s2*(ddq0 + dq0) + c2*dq1*(dq0 + q0)) - c3*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + dq2*(c2*s3*(dq0 + q0) + ...
    c3*s2*(dq0 + q0))) - c1*g - l1*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + l2*(dq1 + dq2)*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0)) +...
    dq1*l1*s2*(dq0 + q0));

t3 = -l2*m2*(l2*(s3*(s2*(ddq0 + dq0) + c2*dq1*(dq0 + q0)) - c3*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + dq2*(c2*s3*(dq0 + q0) +...
    c3*s2*(dq0 + q0))) - c1*g - l1*(c2*(ddq0 + dq0) - dq1*s2*(dq0 + q0)) + l2*(dq1 + dq2)*(c2*s3*(dq0 + q0) + c3*s2*(dq0 + q0)) +...
    dq1*l1*s2*(dq0 + q0));
