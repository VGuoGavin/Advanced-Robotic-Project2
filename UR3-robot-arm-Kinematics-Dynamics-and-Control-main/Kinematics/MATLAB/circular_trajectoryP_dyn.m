%% CIRCULAR TRAJECTORY DESIGNER
%%
% This script computes the trajectory evolution for each joint,
% corresponding to a half circle, with a cubic time law.
% Specifying initial conditions, the final position and orientation and
% final time, it returns, for a cicrular path planning:
% - Position pd(t) [m];
% - Linear Velocity vd(t) [m/s];
% - Linear Acceleration ad(t) [m/s];
% - Orientation phid(t) [rad];
% - Angular Velocity wd(t) [rad/s];
% - Angular Acceleration wd(t) [rad/s];
clear all
%% Input Data
% Initial Conditions
q0 = [1.382 1.2 0.2 -0.5 0.4 0.34]; %q given by the Inverse for p0 and phi0
p0 = [0.09168 -0.5087 0.3532]';
phi0=[-0.8379 1.0919 -0.8474]';
%Final Position
pd=[-0.005695 -0.4587 0.4753]';
phid=[-1.7112  0.95 -1.3979]';
% Final Time
tf = 6;
%% Position Path

t = 0:0.01:tf;
ti=0;
c=p0+(pd-p0)/2;
rho=norm((pd-p0)/2);
x_=(c-p0)/norm(c-p0);
%%Circle Plane%%%%%%%%%%
%%input coordinates:
pv2=1;
pv3=-1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plane_v=[(-x_(2)*pv2-x_(3)*pv3)/x_(1) pv2 pv3]';
z_=plane_v/norm(plane_v);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
y_=cross(z_,x_);
R=[x_ y_ z_];

%Define Time-Law Constants%%%%%%%%%%
norma=rho*pi;
a0=(norma* (3* tf *ti^2 - ti^3))/(tf - ti)^3;
a1=-(6 *norma* tf* ti)/(tf - ti)^3;
a2=(3* norma *(tf + ti))/(tf - ti)^3;
a3=-((2 *norma)/(tf - ti)^3);
s = a0+a1*t+a2*t.^2+a3*t.^3;
s_dot=a1+2*a2*t+3*a3*t.^2;
s_dotdot=2*a2+6*a3*t;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:length(t)
    p_=[rho*cos(s(j)/rho) rho*sin(s(j)/rho) 0]';
    p=c-R*p_;
    p_dot=R*s_dot(j)*[-sin(s(j)/rho) cos(s(j)/rho) 0]';    
    p_dotdot=R*[-s_dot(j)^2*cos(s(j)/rho)/rho-s_dotdot(j)*sin(s(j)/rho) -s_dot(j)^2*sin(s(j)/rho)/rho-s_dotdot(j)*cos(s(j)/rho) 0]';
    
    pdx(j)=p(1);
    pdy(j)=p(2);
    pdz(j)=p(3);
    vdx(j)=p_dot(1);
    vdy(j)=p_dot(2);
    vdz(j)=p_dot(3);
    adx(j)=p_dotdot(1);
    ady(j)=p_dotdot(2);
    adz(j)=p_dotdot(3);
end
%% Orientation Path - Euler Angles

    %Define Time-Law Constants%%%%%%%%%%
    norma=norm(phid-phi0);
    a0=(norma* (3* tf *ti^2 - ti^3))/(tf - ti)^3;
    a1=-(6 *norma* tf* ti)/(tf - ti)^3;
    a2=(3* norma *(tf + ti))/(tf - ti)^3;
    a3=-((2 *norma)/(tf - ti)^3);
    s = a0+a1*t+a2*t.^2+a3*t.^3;
    s_dot=a1+2*a2*t+3*a3*t.^2;
    s_dotdot=2*a2+6*a3*t;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for k=1:length(t)
        phi=phi0+s(k)*(phid-phi0)/norma;
        wd=s_dot(k)*(phid-phi0)/norma;
        wd_dot=s_dotdot(k)*(phid-phi0)/norma;
        
        phidx(k)=phi(1);
        phidy(k)=phi(2);
        phidz(k)=phi(3);
        wdx(k)=wd(1);
        wdy(k)=wd(2);
        wdz(k)=wd(3);
        alphadx(k)=wd_dot(1);
        alphady(k)=wd_dot(2);
        alphadz(k)=wd_dot(3);        
        
    end
