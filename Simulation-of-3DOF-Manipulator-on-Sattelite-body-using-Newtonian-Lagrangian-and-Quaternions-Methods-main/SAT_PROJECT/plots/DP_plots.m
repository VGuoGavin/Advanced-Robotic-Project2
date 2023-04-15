clc;
clear;
close all;
x=load('Lag_var.mat');
y=load('New_var.mat');
t=x.t;

figure
hold on;grid on
plot(t,x.q1,'b-','linewidth',2)
plot(t,y.q1,'g--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('X(m)','fontsize',10,'fontweight','bold');
title('comparison Xg','fontsize',12)

figure
hold on;grid on
plot(t,x.q2,'b-','linewidth',2)
plot(t,y.q2,'g--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('Y(m)','fontsize',10,'fontweight','bold');
title('comparison Yg','fontsize',12)

figure
hold on;grid on
plot(t,x.q3,'b-','linewidth',2)
plot(t,y.q3,'g--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('Z(m)','fontsize',10,'fontweight','bold');
title('comparison Zg','fontsize',12)

figure
hold on;grid on
plot(t,x.q4,'b-','linewidth',2)
plot(t,y.q4,'g--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('\psi (rad)','fontsize',10,'fontweight','bold');
title('comparison \psi','fontsize',12)

figure
hold on;grid on
plot(t,x.q5,'b-','linewidth',2)
plot(t,y.q5,'g--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('\theta (rad)','fontsize',10,'fontweight','bold');
title('comparison \theta','fontsize',12)

figure
hold on;grid on
plot(t,x.q6,'b-','linewidth',2)
plot(t,y.q6,'g--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('\phi (rad)','fontsize',10,'fontweight','bold');
title('comparison \phi','fontsize',12)

figure
hold on;grid on
plot(t,x.q7,'b-','linewidth',2)
plot(t,y.q7,'r--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('\theta_1 (rad)','fontsize',10,'fontweight','bold');
title('comparison \theta_1','fontsize',12)

figure
hold on;grid on
plot(t,x.q8,'b-','linewidth',2)
plot(t,y.q8,'r--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('\theta_2 (rad)','fontsize',10,'fontweight','bold');
title('comparison \theta_2','fontsize',12)

figure
hold on;grid on
plot(t,x.q9,'b-','linewidth',2)
plot(t,y.q9,'r--','linewidth',2)
legend('Lagrange','Newton')
set(gca,'fontsize',8,'fontweight','bold');
xlabel('Time (s)','fontsize',10,'fontweight','bold');
ylabel('\theta_3 (rad)','fontsize',10,'fontweight','bold');
title('comparison \theta_3','fontsize',12)
