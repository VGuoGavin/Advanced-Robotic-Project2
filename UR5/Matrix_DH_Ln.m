function Matrix_DH_Ln(i)     
%这个是D-H矩阵的算法函数
% Caculate the D-H Matrix
global Link

ToDeg = 180/pi;
ToRad = pi/180;

C=cos(Link(i).th);
S=sin(Link(i).th);
Ca=cos(Link(i).alf);
Sa=sin(Link(i).alf);
a=Link(i).dx;    %distance between zi and zi-1
d=Link(i).dz;    %distance between xi and xi-1
y=Link(i).dy;

Link(i).n=[C, S, 0, 0]';
Link(i).o=[-1*S*Ca, C*Ca,Sa, 0]';
Link(i).a=[S*Sa, -1*C*Sa,Ca,0]';
%Link(i).p=[a*C-y*S, a*S+y*C,d,1]';     %书本第57页的D-H矩阵  源代码中的
Link(i).p=[a*C, a*S,d,1]';

Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];%把上面D-H矩阵前面的3*3矩阵存起来
Link(i).A=[Link(i).n,Link(i).o,Link(i).a,Link(i).p];%把第i到i+1的D-H矩阵存进Link(i).A



