%机械臂建模
%暂且只支持改进的DH参数
L1=Link([0,0,0,0,1],'modified');
L2=Link([0,0,0,0,1],'modified');
L3=Link([0,0,0.5,0,0],'modified');
L4=Link([0,-0.01,1,0,0],'modified');
L5=Link([0,0,1,0,0],'modified');
L1.qlim=[0,0];
L2.qlim=[0,0.5];
L3.qlim=[-pi/2,pi/2];
L4.qlim=[-pi,pi/2];
L5.qlim=[-2*pi,2*pi];

L1.I=[0,0,0,0,0,0]; 
L2.I=[0,0,1,0,0,0];
L3.I=[0,0,1,0,0,0];
L4.I=[0,0,1,0,0,0];
L5.I=[0,0,1,0,0,0];


L1.m=0;
L2.m=1;
L3.m=1;
L4.m=1;
L5.m=1;
L1.r=[0;0;0];
L2.r=[0;0;0];
L3.r=[0;0;0];
L4.r=[0;0;0];
L5.r=[0;0;0];
L2.G=2;
L3.G=5;
L4.G=4;
L5.G=4;
L2.Jm=1;
L3.Jm=0.04;
L4.Jm=0.04;
L5.Jm=0.04;
ZRobot=SerialLink([L1,L2,L3,L4,L5]);



