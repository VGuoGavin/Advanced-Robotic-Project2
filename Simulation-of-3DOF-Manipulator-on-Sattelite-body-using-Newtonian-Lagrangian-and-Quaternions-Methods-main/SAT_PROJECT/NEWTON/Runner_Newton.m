clc
clear
close all
%% Newton
addpath('generate51');

global a b c L1 L2 L3 Ms m1 m2 m3 Ixx Ixy Ixz Iyy Iyz Izz tav1 tav2 tav3 fX fY fZ

%P = SAT_PARAMETERS();
a=P.a; b=P.b; c=P.c;
L1=P.L1; L2=P.L2; L3=P.L3;
Ms=P.Ms; m1=P.m1; m2=P.m2; m3=P.m3;
Ixx=P.Ixx; Ixy=P.Ixy; Ixz=P.Ixz; Iyy=P.Iyy; Iyz=P.Iyz; Izz=P.Izz;  
tav1=P.tav1; tav2=P.tav2; tav3=P.tav3;
fX=P.fX; fY=P.fY; fZ=P.fZ;

dt=0.01;
timeSpan=0:dt:5;
tsize=length(timeSpan);

q_0=[0;0;0;0;0;0;0;0;0];
dq_0=[10;-5;0.1;0.2;0.1;0;0;0;0];
init=[q_0;dq_0];

options = odeset('maxstep',0.001);
t0=clock;
[t,z] = ode45(@Newton_Sat_Dynamics,timeSpan,init,options);
Time_New=etime(clock,t0)


q1 = z(:,1); q2 = z(:,2); q3 = z(:,3); q4 = z(:,4); q5 = z(:,5); q6 = z(:,6); q7 = z(:,7); q8 = z(:,8);q9 = z(:,9);
dq1 = z(:,10); dq2 = z(:,11); dq3 = z(:,12); dq4 = z(:,13); dq5 = z(:,14); dq6 = z(:,15); dq7 = z(:,16); dq8 = z(:,17); dq9 = z(:,18);

%% Mechanical Energy
E = Efunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,dq1,dq2,dq3,dq4,dq5,dq6,dq7,dq8,dq9,m1,m2,m3,q4,q5,q6,q7,q8,q9);

ER=(E-E(1))/E(1);
%% ANGULAR AND LINEAR MOMENTUM
% Checksing Linear Momentum Conservation
LM_x = t;
LM_y = t;
LM_z = t;
for i=1:length(t)
    P_XYZ = Pfunc(L1,L2,L3,Ms,c,dq1(i),dq2(i),dq3(i),dq4(i),dq5(i),dq6(i),dq7(i),dq8(i),dq9(i),m1,m2,m3,q4(i),q5(i),q6(i),q7(i),q8(i),q9(i));
    LM_x(i) =  P_XYZ(1);
    LM_y(i) =  P_XYZ(2);    
    LM_z(i) =  P_XYZ(3);
end
%Checksing Angular Momentum Conservation
AM_x = t;
AM_y = t;
AM_z = t;
for i=1:length(t)
    H_XYZ = Hfunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,dq1(i),dq2(i),dq3(i),dq4(i),dq5(i),dq6(i),dq7(i),dq8(i),dq9(i),m1,m2,m3,q1(i),q2(i),q3(i),q4(i),q5(i),q6(i),q7(i),q8(i),q9(i));
    AM_x(i) = H_XYZ(1);    
    AM_y(i) = H_XYZ(2);    
    AM_z(i) = H_XYZ(3);
end

%% PLOTS
SAT_ANIMATION(t,a,b,c,L1,L2,L3,q1,q2,q3,q4,q5,q6,q7,q8,q9)
str='Newtonian Method';
nan = '~';
plotter(str,t,q1,q2,q3,q4,q5,q6,q7,q8,q9,ER,LM_x,LM_y,LM_z,AM_x,AM_y,AM_z)
%% FUNCTIONS
function dz = Newton_Sat_Dynamics(t,z)

dz = z;
dz(1:9) = z(10:18);

global a b c L1 L2 L3 Ms m1 m2 m3 Ixx Ixy Ixz Iyy Iyz Izz tav1 tav2 tav3 fX fY fZ

q1 = z(1); q2 = z(2); q3 = z(3); q4 = z(4); q5 = z(5); q6 = z(6); q7 = z(7); q8 = z(8);q9 = z(9);
dq1 = z(10); dq2 = z(11); dq3 = z(12); dq4 = z(13); dq5 = z(14); dq6 = z(15); dq7 = z(16); dq8 = z(17); dq9 = z(18);

M = Mfunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,m1,m2,m3,q4,q5,q6,q7,q8,q9);

B = Bfunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,dq1,dq2,dq3,dq4,dq5,dq6,dq7,dq8,dq9,fX,fY,fZ,m1,m2,m3,q4,q5,q6,q7,q8,q9,tav1,tav2,tav3);
dz(10:18) = -M\B; 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
end

function SAT_ANIMATION(t,a,b,c,L1,L2,L3,q1,q2,q3,q4,q5,q6,q7,q8,q9)
%ANIMATION PART
hold on;grid on
Len=length(t);
for i=1:Len
    R01 = [cos(q4(i)) sin(q4(i)) 0;-sin(q4(i)) cos(q4(i)) 0;0 0 1];%XYZ----->x1y1z1
    R12 = [cos(q5(i)) 0 -sin(q5(i));0 1 0;sin(q5(i)) 0 cos(q5(i))];%x1y1z1----->x2y2z2
    R23 = [1 0 0;0 cos(q6(i)) sin(q6(i));0 -sin(q6(i)) cos(q6(i))];%x2y2z2----->x3y3z3
    R03 = R23*R12*R01;                                              %XYZ----->x3y3z3
    R34 = [cos(q7(i)) sin(q7(i)) 0;-sin(q7(i)) cos(q7(i)) 0;0 0 1]; %x3y3z3----->x3y3z3
    R45 = [cos(q8(i)) 0 -sin(q8(i));0 1 0;sin(q8(i)) 0 cos(q8(i))]; %x4y4z4----->x5y5z5
    R56 = [cos(q9(i)) 0 -sin(q9(i));0 1 0;sin(q9(i)) 0 cos(q9(i))]; %x5y5z5----->x6y6z6

    
    Rt1=R03.';
    Rt2=(R45*R34*R03).';
    Rt3=(R56*R45*R34*R03).';
    %Satellite Corners
    P1 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[-b;-a;c];
    P2 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[b;-a;c];
    P3 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[-b;a;c];
    P4 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[b;a;c];
    P5 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[b;a;-c];
    P6 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[b;-a;-c];
    P7 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[-b;a;-c];
    P8 = [q1(i);q2(i);q3(i)]+Rt1*1/2*[-b;-a;-c];    
    %Manipulators Joints
    J1 = [q1(i);q2(i);q3(i)]+Rt1*[0;0;c/2];
    J2 = [q1(i);q2(i);q3(i)]+Rt1*[0;0;c/2+L1];
    J3 = J2+Rt2*[0;L2;0];
    END = J3+Rt3*[0;L3;0];
    %
    V12 = [P1,P2]; V13 = [P1,P3]; V24 = [P2,P4]; V34 = [P3,P4]; 
    V45 = [P4,P5]; V56 = [P5,P6]; V57 = [P5,P7]; V78 = [P7,P8]; 
    V18 = [P1,P8]; V37 = [P3,P7]; V26 = [P2,P6]; V86 = [P8,P6];
    %
    VJ1J2 = [J1,J2];
    VJ2J3 = [J2,J3];
    VJ3END = [J3,END];
    %
    plot3(V12(1,:),V12(2,:),V12(3,:),'b','linewidth',5);title('MANIPULATOR ON SATTELITE')
    xlabel('Xg');ylabel('Yg');zlabel('Zg');
    hold on;grid on
    plot3(V13(1,:),V13(2,:),V13(3,:),'b-','linewidth',5)
    plot3(V24(1,:),V24(2,:),V24(3,:),'b-','linewidth',5)
    plot3(V34(1,:),V34(2,:),V34(3,:),'b-','linewidth',5)
    plot3(V45(1,:),V45(2,:),V45(3,:),'b-','linewidth',5)
    plot3(V56(1,:),V56(2,:),V56(3,:),'b-','linewidth',5)
    plot3(V57(1,:),V57(2,:),V57(3,:),'b-','linewidth',5)
    plot3(V78(1,:),V78(2,:),V78(3,:),'b-','linewidth',5)
    plot3(V18(1,:),V18(2,:),V18(3,:),'b-','linewidth',5)
    plot3(V37(1,:),V37(2,:),V37(3,:),'b-','linewidth',5)
    plot3(V26(1,:),V26(2,:),V26(3,:),'b-','linewidth',5)
    plot3(V86(1,:),V86(2,:),V86(3,:),'b-','linewidth',5) 
    plot3(V26(1,:),V26(2,:),V26(3,:),'b-','linewidth',5)
    plot3(V45(1,:),V45(2,:),V45(3,:),'b-','linewidth',5)   
    
        
    plot3(VJ1J2(1,:),VJ1J2(2,:),VJ1J2(3,:),'r-','linewidth',4)
    plot3(VJ2J3(1,:),VJ2J3(2,:),VJ2J3(3,:),'g-','linewidth',3)
    plot3(VJ3END(1,:),VJ3END(2,:),VJ3END(3,:),'y-','linewidth',2)

    axis equal
    axis([q1(i)-2*a  (q1(i)+2*a) (q2(i)-2*a) (q2(i)+2*a)  (q3(i)-2*a) (q3(i)+2*a)])
    str=['Time = ',num2str(t(i))];
    text(q1(i)+a,q2(i)-a,q3(i)-2*a,str)
    
    hold off
    pause(0.01)
end
end

function plotter(str,t,q1,q2,q3,q4,q5,q6,q7,q8,q9,ER,LM_x,LM_y,LM_z,AM_x,AM_y,AM_z)
    figure
    hold on;grid on
    plot(t,q1,'b-','linewidth',2)
    plot(t,q2,'r-','linewidth',2)
    plot(t,q3,'g--','linewidth',2)
    legend('X','Y','Z')
    set(gca,'fontsize',8,'fontweight','bold');
    xlabel('Time (s)','fontsize',10,'fontweight','bold');
    ylabel('Satellite Global Position (m)','fontsize',10,'fontweight','bold');
    title(str,'fontsize',12)

    figure
    hold on;grid on
    plot(t,q4,'b-','linewidth',2)
    plot(t,q5,'r-','linewidth',2)
    plot(t,q6,'g-','linewidth',2)
    legend('\psi','\theta','\phi')
    set(gca,'fontsize',8,'fontweight','bold');
    xlabel('Time (s)','fontsize',10,'fontweight','bold');
    ylabel('Satellite Euler Angles (rad)','fontsize',10,'fontweight','bold');
    title(str,'fontsize',12)

    figure
    hold on;grid on
    plot(t,q7,'b-','linewidth',2)
    plot(t,q8,'r-','linewidth',2)
    plot(t,q9,'g-','linewidth',2)
    legend('\theta_1','\theta_2','\theta_3')
    set(gca,'fontsize',8,'fontweight','bold');
    xlabel('Time (s)','fontsize',10,'fontweight','bold');
    ylabel('Manipulator link Angles (rad)','fontsize',10,'fontweight','bold');
    title(str,'fontsize',12)
    
    if ER ~= '~'
        figure
        grid on
        plot(t,ER,'g-','linewidth',2)
        set(gca,'fontsize',8,'fontweight','bold');
        xlabel('Time (s)','fontsize',10,'fontweight','bold');
        ylabel('Mechanical Energy Error (J)','fontsize',10,'fontweight','bold');
        title(str,'fontsize',12)
    end
    if LM_x ~= '~'
        figure
        hold on;grid on
        plot(t,(LM_x-LM_x(1))/LM_x(1),'b-','linewidth',2)
        plot(t,(LM_y-LM_y(1))/LM_y(1),'r--','linewidth',2)
        plot(t,(LM_z-LM_z(1))/LM_z(1),'g-','linewidth',2)
        set(gca,'fontsize',8,'fontweight','bold');
        ylabel('Relative Error of Linear Momentum % ','fontsize',10,'fontweight','bold');
        xlabel('Time (s)','fontsize',10,'fontweight','bold');
        legend('P_X','P_Y','P_Z')
        title(str,'fontsize',12)
    end
    if AM_x ~= '~'
        figure
        hold on;grid on
        plot(t,(AM_x-AM_x(1))/AM_x(1),'b-','linewidth',2)
        plot(t,(AM_y-AM_y(1))/AM_y(1),'r-','linewidth',2)
        plot(t,(AM_z-AM_z(1))/AM_z(1),'g-','linewidth',2)
        set(gca,'fontsize',8,'fontweight','bold');
        ylabel('Relative Error of Angular Momentum % ','fontsize',10,'fontweight','bold');
        xlabel('Time (s)','fontsize',10,'fontweight','bold');
        legend('H_X','H_Y','H_Z')
        title(str,'fontsize',12)
    end
end