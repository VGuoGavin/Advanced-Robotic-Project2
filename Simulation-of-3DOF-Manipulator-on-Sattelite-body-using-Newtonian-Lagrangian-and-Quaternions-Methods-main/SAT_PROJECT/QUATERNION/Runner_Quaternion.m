clc
clear
close all
%% Quaternions
addpath('generate71');

global a b c L1 L2 L3 Ms m1 m2 m3 Ixx Ixy Ixz Iyy Iyz Izz tav1 tav2 tav3 fX fY fZ

%P = SAT_PARAMETERS();
%P = SAT_PARAMETERS();
a=1; b=1; c=1;
L1=1; L2=1; L3=1;
Ms=1; m1=1; m2=1; m3=1;
Ixx=1; Ixy=1; Ixz=1; Iyy=1; Iyz=1; Izz=1;  
tav1=1; tav2=1; tav3=1;
fX=1; fY=1; fZ=1;



q0 = [0;0;0;0;0;0;0;0;0];
dq0 = [10;-5;0.1;0.2;0.1;0;0;0;0];
z0=[q0;dq0];

z00(1:3)=z0(1:3);
z00(4:7)=angle2quat(z0(4),z0(5),z0(6));
z00(8:13)=z0(7:12);

w_dq =[         -sin(z0(5)),           0, 1;
        cos(z0(5))*sin(z0(6)),  cos(z0(6)), 0;
        cos(z0(5))*cos(z0(6)), -sin(z0(6)), 0];
    
w01=w_dq*z0(13:15);

e0=z00(4); e1=z00(5); e2=z00(6); e3=z00(7);
z00(14:17)=0.5*[-e1 -e2 -e3;e0 -e3 e2;e3 e0 -e1;-e2 e1 e0]*w01;

z00(18:20)=z0(16:18);

timespan=0:0.01:5;
options=odeset('maxstep',0.001);
t0=clock;
[t,Z]=ode45(@dyn_Quat,timespan,z00,options);
Time_Quat=etime(clock,t0)

q1 = Z(:,1);q2 = Z(:,2);q3 = Z(:,3);e0=Z(:,4); e1=Z(:,5); e2=Z(:,6); e3=Z(:,7);q7 = Z(:,8);q8 = Z(:,9);q9 = Z(:,10);
dq1 = Z(:,11);dq2 = Z(:,12);dq3 = Z(:,13);de0=Z(:,14); de1=Z(:,15); de2=Z(:,16); de3=Z(:,17);dq7 = Z(:,18);dq8 = Z(:,19);dq9 = Z(:,20);

q4=t;q5=t;q6=t;
for i=1:length(t)    
    [q4(i), q5(i), q6(i)] = quat2angle([e0(i),e1(i),e2(i),e3(i)]);
end

%% Mechanical Energy
E = EFunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,de0,de1,de2,de3,dq1,dq2,dq3,dq7,dq8,dq9,e0,e1,e2,e3,m1,m2,m3,q7,q8,q9);
 
ER=(E-E(1))/E(1);
%% ANGULAR AND LINEAR MOMENTUM
% Checksing Linear Momentum Conservation
LM_x = t;
LM_y = t;
LM_z = t;
for i=1:length(t)
    P_XYZ = PFunc(L1,L2,L3,Ms,c,de0(i),de1(i),de2(i),de3(i),dq1(i),dq2(i),dq3(i),dq7(i),dq8(i),dq9(i),e0(i),e1(i),e2(i),e3(i),m1,m2,m3,q7(i),q8(i),q9(i));
    LM_x(i) =  P_XYZ(1);
    LM_y(i) =  P_XYZ(2);    
    LM_z(i) =  P_XYZ(3);
end
%Checksing Angular Momentum Conservation
AM_x = t;
AM_y = t;
AM_z = t;
for i=1:length(t)
    H_XYZ = HFunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,de0(i),de1(i),de2(i),de3(i),dq1(i),dq2(i),dq3(i),dq7(i),dq8(i),dq9(i),e0(i),e1(i),e2(i),e3(i),m1,m2,m3,q1(i),q2(i),q3(i),q7(i),q8(i),q9(i));
    AM_x(i) = H_XYZ(1);    
    AM_y(i) = H_XYZ(2);    
    AM_z(i) = H_XYZ(3);
end

%% PLOTS
SAT_ANIMATION(t,a,b,c,L1,L2,L3,q1,q2,q3,q4,q5,q6,q7,q8,q9)
str='Quaternions Method';
nan = '~';
plotter(str,t,q1,q2,q3,q4,q5,q6,q7,q8,q9,ER,LM_x,LM_y,LM_z,AM_x,AM_y,AM_z)

%% FUNCTIONS
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

function dZ=dyn_Quat(t,Z)

dZ=Z;

global a b c L1 L2 L3 Ms m1 m2 m3 Ixx Ixy Ixz Iyy Iyz Izz tav1 tav2 tav3 fX fY fZ

q1 = Z(1);q2 = Z(2);q3 = Z(3);e0=Z(4); e1=Z(5); e2=Z(6); e3=Z(7);q7 = Z(8);q8 = Z(9);q9 = Z(10);
dq1 = Z(11);dq2 = Z(12);dq3 = Z(13);de0=Z(14); de1=Z(15); de2=Z(16); de3=Z(17);dq7 = Z(18);dq8 = Z(19);dq9 = Z(20);

M = MFunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,e0,e1,e2,e3,m1,m2,m3,q7,q8,q9);
B = BFunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,c,de0,de1,de2,de3,dq1,dq2,dq3,dq7,dq8,dq9,e0,e1,e2,e3,m1,m2,m3,q7,q8,q9);
Q = [fX;fY;fZ;-tav1;0;0;-tav1;tav1;tav2-tav3;tav3];

a1=[0 0 0 e0 e1 e2 e3 0 0 0];
a1_dot=[0 0 0 de0 de1 de2 de3 0 0 0];

F_aug=[-(B+Q);a1_dot*Z(11:20)];
M_aug=[M -a1.';-a1 0];

dZ(1:10)=Z(11:20);

dzz=M_aug\F_aug;

dZ(11:20)=dzz(1:10);

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
        plot(t,(LM_x-LM_x(1))/LM_x(1)*100,'b-','linewidth',2)
        plot(t,(LM_y-LM_y(1))/LM_y(1)*100,'r-','linewidth',2)
        plot(t,(LM_z-LM_z(1))/LM_z(1)*100,'g--','linewidth',2)
        set(gca,'fontsize',8,'fontweight','bold');
        ylabel('Relative Error of Linear Momentum % ','fontsize',10,'fontweight','bold');
        xlabel('Time (s)','fontsize',10,'fontweight','bold');
        legend('P_X','P_Y','P_Z')
        title(str,'fontsize',12)
    end
    if AM_x ~= '~'
        figure
        hold on;grid on
        plot(t,(AM_x-AM_x(1))/AM_x(1)*100,'b-','linewidth',2)
        plot(t,(AM_y-AM_y(1))/AM_y(1)*100,'r-','linewidth',2)
        plot(t,(AM_z-AM_z(1))/AM_z(1)*100,'g-','linewidth',2)
        set(gca,'fontsize',8,'fontweight','bold');
        ylabel('Relative Error of Angular Momentum % ','fontsize',10,'fontweight','bold');
        xlabel('Time (s)','fontsize',10,'fontweight','bold');
        legend('H_X','H_Y','H_Z')
        title(str,'fontsize',12)
    end
end




