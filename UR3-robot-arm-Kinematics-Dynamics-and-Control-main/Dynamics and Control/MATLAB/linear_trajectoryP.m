%% LINEAR TRAJECTORY DESIGNER
%%
% This script computes the trajectory evolution for each joint, based in
% the interpolation of some positions and rotations of reference, with a
% cubic time law. Specifying initial conditions, each pose's position and
% orientation and final time, it returns, for a linear path planning:
% - Position pd(t) [mm];
% - Linear Velocity vd(t) [mm/s];
% - Orientation phid(t) [rad];
% - Angular Velocity wd(t) [rad/s];
%clear all
%% Input Data
% Initial Conditions
q0 = [1.382 1.2 0.2 -0.5 0.4 0.34]';
% Pose 1
p0=[0.09168 -0.5087 0.2012]';
phi0=[-0.8379 1.0919 -0.8474]';
%Pose 2
pd(:,1)=[0.09168 -0.5087 -0.5]';
phid(:,1)=[-1 1 -0.8]';
%Pose 3
pd(:,2)=[0.09168 0.2 -0.5]';
phid(:,2)=[-0.8379 1.0919 -0.8474]';
% Final Times (per path)
tf1 = 1;
tf2 = 2;
tf = tf2;

%% Position Path

t = 0:0.01:tf;

for j=1:length(t)
    
    
    if 0<=t(j) && t(j)<=tf1
        %Define Time-Law Constants%%%%%%%%%%
        norma=norm(pd(:,1)-p0);
        ti=0;
        tf=tf1;
        a0=(norma* (3* tf *ti^2 - ti^3))/(tf - ti)^3;
        a1=-(6 *norma* tf* ti)/(tf - ti)^3;
        a2=(3* norma *(tf + ti))/(tf - ti)^3;
        a3=-((2 *norma)/(tf - ti)^3);
        s = a0+a1*t+a2*t.^2+a3*t.^3;
        s_dot=a1+2*a2*t+3*a3*t.^2;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        pd_int = p0+s(j)*(pd(:,1)-p0)/norma;
        vd = s_dot(j)*(pd(:,1)-p0)/norma;
    elseif tf1<t(j) && t(j)<=tf2
        %Define Time-Law Constants%%%%%%%%%%
        norma=norm(pd(:,2)-pd(:,1));
        ti=tf1;
        tf=tf2;
        a0=(norma* (3* tf *ti^2 - ti^3))/(tf - ti)^3;
        a1=-(6 *norma* tf* ti)/(tf - ti)^3;
        a2=(3* norma *(tf + ti))/(tf - ti)^3;
        a3=-((2 *norma)/(tf - ti)^3);
        s = a0+a1*t+a2*t.^2+a3*t.^3;
        s_dot=a1+2*a2*t+3*a3*t.^2;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        pd_int = pd(:,1)+(s(j))*(pd(:,2)-pd(:,1))/norma;
        vd = s_dot(j)*(pd(:,2)-pd(:,1))/norma;
    end
    pdx(j)=pd_int(1);
    pdy(j)=pd_int(2);
    pdz(j)=pd_int(3);
    vdx(j)=vd(1);
    vdy(j)=vd(2);
    vdz(j)=vd(3);
end
%% Orientation Path - Euler Angles (linear path)

for k=1:length(t)
    if 0<=t(k) && t(k)<=tf1
        %Define Time-Law Constants%%%%%%%%%%
        norma=norm(phid(:,1)-phi0);
        ti=0;
        tf=tf1;
        a0=(norma* (3* tf *ti^2 - ti^3))/(tf - ti)^3;
        a1=-(6 *norma* tf* ti)/(tf - ti)^3;
        a2=(3* norma *(tf + ti))/(tf - ti)^3;
        a3=-((2 *norma)/(tf - ti)^3);
        s = a0+a1*t+a2*t.^2+a3*t.^3;
        s_dot=a1+2*a2*t+3*a3*t.^2;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        phi=phi0+s(k)*(phid(:,1)-phi0)/norma;
        wd=s_dot(k)*(phid(:,1)-phi0)/norma;
    elseif tf1<t(k) && t(k)<=tf2
        %Define Time-Law Constants%%%%%%%%%%
        norma=norm(phid(:,2)-phid(:,1));
        ti=tf1;
        tf=tf2;
        a0=(norma* (3* tf *ti^2 - ti^3))/(tf - ti)^3;
        a1=-(6 *norma* tf* ti)/(tf - ti)^3;
        a2=(3* norma *(tf + ti))/(tf - ti)^3;
        a3=-((2 *norma)/(tf - ti)^3);
        s = a0+a1*t+a2*t.^2+a3*t.^3;
        s_dot=a1+2*a2*t+3*a3*t.^2;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        phi=phid(:,1)+s(k)*(phid(:,2)-phid(:,1))/norma;
        wd=s_dot(k)*(phid(:,2)-phid(:,1))/norma;
    end
    phidx(k)=phi(1);
    phidy(k)=phi(2);
    phidz(k)=phi(3);
    wdx(k)=wd(1);
    wdy(k)=wd(2);
    wdz(k)=wd(3);
end
