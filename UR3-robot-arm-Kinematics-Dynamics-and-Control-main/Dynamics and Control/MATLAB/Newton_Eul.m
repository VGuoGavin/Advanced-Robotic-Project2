function [] = Newton_Eul()

%manipulator characteristics
NDOF = 6;

%% inicial conditions
%velocity and acelerations of the base frame
ddp0 = [0 0 g].';     

w0 = 0;
dw0 = 0;

%rotation matrices
for j = 1:6
    T(j)=DKin(Robot(j,:));
end

% z^i-1_i
zi1_i= [0 0 1].';

%vector from origin of Frame (i ? 1) to origin of Frame i
% Vector r(vector from origin of Frame i to centre of mass Ci)
for K = 1:NDOF
    ri_i1_ci (:,K) = di(K) * ri_i1_ci_Matrix(K,:).' ; 
end

%% Forward Recursion
%First step

Forward = 1;

w(1:3,1) = (T{1}.')*(w0 + dq(1)*z0);

dw (1:3,1) = (T{1}.')*(dw0 + ddq(1)*z0 + cross(dq(1)*w0,z0));

ddp (1:3,1) =(T{1}.')*ddp0_g0 + cross(dw(:,1),ri_i1_ci(:,1)) + cross(w(1:3,1),cross(w(1:3,1),ri_i1_ci(:,1)));

ddpC (1:3,1) = ddp(:,1) + cross(dw(:,1),ri_ici(:,1)) + cross(w(:,1),cross(w(:,1), ri_ici(:,1)));

dwm (1:3,1) = dw0 + Kr(1)*ddq(1)*z0 + Kr(1)*dq(1)*cross(w0,z0);

%Following Recursion
for K = 2:N_DOF
    Forward = Forward + 1;
    
    w(1:3,K) =(T{K}.')*(w(:,K-1) + dq(K)*z0);
    
    dw (1:3,K) = (T{K}.')*(dw(:,K-1) + ddq(K)*z0 + cross(dq(K)*w(:,K-1),z0));
    
    ddp (1:3,K) =(T{K}.')*ddp(:,K-1) + cross(dw(:,K),ri_i1_ci(:,K)) + cross(w(1:3,K),cross(w(1:3,K),ri_i1_ci(:,K)));
    
    ddpC (1:3,K) = ddp(:,K) + cross(dw(:,K),ri_ici(:,K)) + cross(w(:,K),cross(w(:,K),ri_ici(:,K)));
    
    dwm (1:3,K) = dw(:,K-1) + Kr(K)*ddq(K)*zi1_i + Kr(K)*dq(K)*cross(w(:,K-1),zi1_i);
end

%% Backwards recursion
%First step
Backward = N_DOF;

force = sym(zeros(3,N_DOF));
miu = sym(zeros(3,N_DOF));

%Following recursion
k = N_DOF-1:-1:1
    Backward = Backward - 1
    %(7.112)
    force (1:3,k) = R{k+1} * force (1:3,k+1) + DP(k).m * ddpC(:,k);
    %(7.113)
    miu(1:3,k) = cross(-force (:,k),(ri_i1_ci(:,k) + ri_ici(:,k))) ...
        + R{k+1}*miu(:,k+1) ...
        + cross(R{k+1}*force (:,k+1),ri_ici(:,k)) + DP(k).Il * dw(:,k) ...
        + cross(w(:,k),(DP(k).Il * w(:,k))) ...
        + Kr(k+1)*ddq(k+1)*DP(k+1).Im * zi1_i ...
        + Kr(k+1)*dq(k+1)*DP(k+1).Im * cross(w(:,k),zi1_i);
    %(7.114)
    Torque(k) = miu(:,k).'*R{k}.'*z0 + Kr(k)* DP(k).Im*dwm(:,k).'*zim1_mi;
end
%-----------------------------------------------------------------------------

% vector r do referencial i para o Centro de massa do augmented link
% r^i_i,Ci
rCi_Matrix = [0 -1 0; 0 0 1; 0 1 0; 0 0 1; 0 -1 0; 0 0 1; 0 0 -1];
for K = 1:5
    if mod(K,2) == 1  % i impar
        ri_iCi(:,K) = DP(K).r_cl * rCi_Matrix(K,:).';
    end
    if mod(K,2) == 0  % i par
        r_cl = di(K+1)/2 - DP(K).r_cl;
        ri_iCi(:,K) = r_cl * rCi_Matrix(K,:).';
    end
end
ri_iCi(:,N_DOF-1) = (0.078 - DP(N_DOF-1).r_cl)*rCi_Matrix(N_DOF-1,:).';
ri_iCi(:,N_DOF) = 0.015/2*rCi_Matrix(N_DOF,:).';