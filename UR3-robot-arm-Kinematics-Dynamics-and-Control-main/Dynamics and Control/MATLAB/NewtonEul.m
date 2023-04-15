function [G,C_dq,B] = NewtonEul()

%% Definição e Determinação de Parâmetros necessários ---------------------
[dyn]=dparameters();
Robot = UR3;

% Number of Joints - All Revolute
N_DOF = 6; 
for ii = 1:N_DOF
    T(:,:,ii) = DKin(Robot(ii,:));
end
% Base
z0 = [0 0 1].';
w0 = [0 0 0].';
dw0 = [0 0 0].';
ddp0_g0 = [0 0 9.81].';

% Reduction Ratio
Kr = [100 100 100 100 100 100 100];

% % ------------- Características do Manipulador
% di = [0.2 0 0.4 0 0.4 0 0.078]; % Distâncias


% % % -------------- Variáveis Simbólicas
q = sym('q',[N_DOF,1]);
dq = sym('dq',[N_DOF,1]);
ddq = sym('ddq',[N_DOF,1]);

% % --------------- Vector Z do motor i+1 no referencial i sempre igual
% z^i-1_mi
zim1_mi = [0 0 0 0 0 0; 0 0 0 0 0 0; 1 1 -1 1 1 1];

% % --------------- Vectores r
% r^i_i-1,i
% ri_im1ci_Matrix = [0 1 0; 0 0 0; 0 -1 0; 0 0 0; 0 1 0; 0 0 0; 0 0 1];

for j = 1:N_DOF
    ri_im1ci (:,j) = (T(1:3,1:3,j)')*T(1:3,4,j); % Vector r
end

% vector r do referencial i para o Centro de massa do augmented link
% r^i_i,Ci
% rCi_Matrix = [0 -1 0; 0 0 1; 0 1 0; 0 0 1; 0 -1 0; 0 0 1; 0 0 -1];
% for j = 1:5
%     if mod(j,2) == 1  % i impar
%         ri_iCi(:,j) = dyn(j).r_cl * rCi_Matrix(j,:).';
%     end
%     if mod(j,2) == 0  % i par
%         r_cl = di(j+1)/2 - dyn(j).r_cl;
%         ri_iCi(:,j) = r_cl * rCi_Matrix(j,:).';
%     end
% end
% ri_iCi(:,N_DOF-1) = (0.078 - dyn(N_DOF-1).r_cl)*rCi_Matrix(N_DOF-1,:).';
% ri_iCi(:,N_DOF) = 0.015/2*rCi_Matrix(N_DOF,:).';
for zz = 1:N_DOF
    ri_iCi (:,zz) = dyn(zz).r_cl;
end
%% Forward Recursion ----------------------------------------------------------------
%% Inicialização
Forward = 1;
%(7.107)
w(1:3,1) = T(1:3,1:3,1).'*(w0 + dq(1)*z0);
%(7.108)
dw (1:3,1) =T(1:3,1:3,1).'*(dw0 + ddq(1)*z0 + cross(dq(1)*w0,z0));
%(7.109)
ddp (1:3,1) = T(1:3,1:3,1).'*ddp0_g0 + cross(dw(:,1),ri_im1ci(:,1)) ...
    + cross(w(1:3,1),cross(w(1:3,1),ri_im1ci(:,1)));
%(7.110)
ddpC (1:3,1) =ddp(:,1) + cross(dw(:,1),ri_iCi(:,1)) ...
    + cross(w(:,1),cross(w(:,1), ri_iCi(:,1)));
%(7.111)
dwm (1:3,1) =dw0 + Kr(1)*ddq(1)*z0 + Kr(1)*dq(1)*cross(w0,z0);

%% Recursão

for i = 2:N_DOF
    Forward = Forward + 1;
    %(7.107)
    w(1:3,i) =T(1:3,1:3,i).'*(w(:,i-1) + dq(i)*z0);
    %(7.108)
    dw (1:3,i) = T(1:3,1:3,i).'*(dw(:,i-1) + ddq(i)*z0 ...
        + cross(dq(i)*w(:,i-1),z0));
    %(7.109)
    ddp (1:3,i) =T(1:3,1:3,i).'*ddp(:,i-1) + cross(dw(:,i),ri_im1ci(:,i)) ...
        + cross(w(1:3,i),cross(w(1:3,i),ri_im1ci(:,i)));
    %(7.110)
    ddpC (1:3,i) = ddp(:,i) + cross(dw(:,i),ri_iCi(:,i)) ...
        + cross(w(:,i),cross(w(:,i),ri_iCi(:,i)));
    %(7.111)
    dwm (1:3,i) = dw(:,i-1) + Kr(i)*ddq(i)*zim1_mi(:,i) ...
        + Kr(i)*dq(i)*cross(w(:,i-1),zim1_mi(:,i));
end

%% Backward Recursion --------------------------------------------------------
%% Inicialização
Backward = N_DOF;

f = sym(zeros(3,N_DOF));
mu= sym(zeros(3,N_DOF));

%(7.112)
f (1:3,N_DOF) = dyn(N_DOF).m * ddpC(:,N_DOF);
%(7.113)
mu(1:3,N_DOF) = cross(-f (:,N_DOF),(ri_im1ci(:,N_DOF) + ri_iCi(:,N_DOF))) + dyn(N_DOF).Il * dw(:,N_DOF) + cross(w(:,N_DOF),(dyn(N_DOF).Il * w(:,N_DOF)));
%(7.114)
Torque(N_DOF) = mu(:,N_DOF).'*T(1:3,1:3,N_DOF).'*z0 + Kr(N_DOF)* dyn(N_DOF).Im * dwm(:,N_DOF).'*zim1_mi(:,N_DOF);

%% Recursão
for i = N_DOF-1:-1:1
    Backward = Backward - 1;
    %(7.112)
    f (1:3,i) = T(1:3,1:3,i+1) * f (1:3,i+1) + dyn(i).m * ddpC(:,i);
    %(7.113)
    mu(1:3,i) = cross(-f (:,i),(ri_im1ci(:,i) + ri_iCi(:,i))) ...
        + T(1:3,1:3,i+1)*mu(:,i+1) ...
        + cross(T(1:3,1:3,i+1)*f (:,i+1),ri_iCi(:,i)) + dyn(i).Il * dw(:,i) ...
        + cross(w(:,i),(dyn(i).Il * w(:,i))) ...
        + Kr(i+1)*ddq(i+1)*dyn(i+1).Im * zim1_mi(:,i) ...
        + Kr(i+1)*dq(i+1)*dyn(i+1).Im * cross(w(:,i),zim1_mi(:,i));
    %(7.114)
    Torque(i) = mu(:,i).'*T(1:3,1:3,i).'*z0 + Kr(i)* dyn(i).Im*dwm(:,i).'*zim1_mi(:,i);
end


G = subs(Torque,[dq,ddq],zeros(N_DOF,2));
B = jacobian (Torque,ddq);
C_dq = subs(Torque,ddq,zeros(N_DOF,1)) - G;


