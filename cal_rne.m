function [rne_tau,link]=cal_rne(robot,neq,neqd,neqdd,motor_position)
if nargin==5
    [rne_tau,link]=cal_rne_m(robot,neq,neqd,neqdd,motor_position);
elseif nargin==4
    [rne_tau,link]=cal_rne_nom(robot,neq,neqd,neqdd);
end
end
%考虑电机转子
function [rne_tau,link]=cal_rne_m(robot,neq,neqd,neqdd,motor_position)
neq_temp.q=neq;%关节角度，相对于上一个连杆
neq_temp.qd=neqd;%关节角速度，相对于上一个连杆
neq_temp.qdd=neqdd;%关节角加速度，相对于上一个连杆
%link初始化，link(1)表示机械臂的基座
link.w=zeros(3,1);%连杆角速度，相对于地球坐标系
link.wd=zeros(3,1);%连杆角加速度，相对于地球坐标系
link.wmd=zeros(3,1);%驱动该连杆的电机转子的角加速度，相对于地球坐标系
link.p=zeros(3,1);%连杆坐标系原点的位置，连杆坐标系
% link.a=robot.gravity;
link.a=zeros(3,1);%连杆坐标系原点的线加速度，地球坐标系
link.ac=zeros(3,1);%连杆重心的线加速度，在地球坐标系
link.f=zeros(3,1);%连杆坐标系原点的受力，相对于连杆坐标系
link.tau=0;%连杆坐标系原点的力矩
link.taum=0;%驱动该连杆的电机的力矩
link.pc=zeros(3,1);%连杆的质心的位置，在连杆坐标系中
link.m=0;%连杆的质量
link.I=zeros(3,3);%连杆的惯量矩阵，相对于质心
link.F=zeros(3,1);%连杆的惯性力大小，连杆坐标系
link.N=zeros(3,1);%连杆的惯性力矩，连杆坐标系
link.nm=zeros(3,1);%安装在连杆i上的电机转子转动引起的关节i处的附加力矩
link.nj=zeros(3,1);%不考虑电机转动关节i处的力矩
link.njm=zeros(3,1);%link.nm+link.nj
link.Im=zeros(3,3);%电机转子惯量矩阵
link.G=0;%驱动连杆电机的减速比
link.zm=[0;0;1];%连杆i的电机旋转轴方向在连杆i坐标系的表示
link(2).a=robot.gravity;
z0=[0;0;1];%Z方向向量
link_num_temp=size(robot.links);%连杆数量
link_num=link_num_temp(2);%连杆数量
T(1:4,1:4,1)=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%T矩阵
%赋值减速比和电机转子的转动惯量
for i=2:link_num
    if isempty(robot.links(1,i).G)==0
        link(i).G=robot.links(1,i).G;
    else
        link(i).G=0;
    end
    if isempty(robot.links(1,i).Jm)==0
        link(i).Im=[0 0 0;0 0 0;0 0 robot.links(1,i).Jm];
    else
        link(i).Im=zeros(3,3);
    end
end
%计算T矩阵 根据DH参数不同和连杆的关节类型而不同
if robot.mdh==1 %如果是改进的DH参数
    for i=2:link_num
        if robot.links(1,i).sigma==1
            robot.links(1,i).d=neq_temp.q(i);
        else
            robot.links(1,i).theta=neq_temp.q(i);
        end
        T(1:4,1:4,i)=T(1:4,1:4,i-1)*[
            cos(robot.links(1,i).theta),-sin(robot.links(1,i).theta),0,robot.links(1,i).a;
            sin(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),cos(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),-sin(robot.links(1,i).alpha),-sin(robot.links(1,i).alpha)*robot.links(1,i).d;
            sin(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),cos(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),cos(robot.links(1,i).alpha),cos(robot.links(1,i).alpha)*robot.links(1,i).d;
            0,0,0,1];
        link(i).p=T(1:3,4,i);
    end
else%如果是标准的DH参数 
    for i=2:link_num
        if robot.links(1,i).sigma==1
            robot.links(1,i).d=neq_temp.q(i);
        else
            robot.links(1,i).theta=neq_temp.q(i);
        end
        T(1:4,1:4,i)=T(1:4,1:4,i-1)*[
            cos(robot.links(1,i).theta),-sin(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),sin(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),robot.links(1,i).a*cos(robot.links(1,i).theta);
            sin(robot.links(1,i).theta),cos(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),-cos(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),robot.links(1,i).a*sin(robot.links(1,i).theta);
            0,sin(robot.links(1,i).alpha),cos(robot.links(1,i).alpha),robot.links(1,i).d;
            0,0,0,1];
        link(i).p=T(1:3,4,i);
    end
end
%求连杆坐标系原点的线速度、连杆坐标系原点的线加速度、连杆的角速度、连杆的角加速度、质心的线加速度、电机转子的角加速度
for i=2:1:link_num %从连杆2开始，
    R=T(1:3,1:3,i)'*T(1:3,1:3,i-1);% i-1在i中的旋转矩阵 R 上i下i-1
    P=T(1:3,1:3,i-1)'*(T(1:3,4,i)-T(1:3,4,i-1));% ；i坐标系原点在i-1坐标系中的位置 P上i-1下i
    link(i).pc=robot.links(1,i).r';
    link(i).I=robot.links(1,i).I;
    link(i).m=robot.links(1,i).m;
    if robot.links(i).sigma==1  %若为移动关节
        link(i).w=R*link(i-1).w;%连杆的角速度
        link(i).wd=R*link(i-1).w;%连杆的角加速度
        a_temp1=R*(cross(link(i-1).wd,P)+cross(link(i-1).w,cross(link(i-1).w,P))+link(i-1).a);
        a_temp2=2*cross(link(i).w,(neq_temp.qd(i)*z0));
        a_temp3=neq_temp.qdd(i)*z0;
        link(i).a=a_temp1+a_temp2+a_temp3;%连杆坐标系原点的线速度
    else  %若为旋转关节
        link(i).w=R*(link(i-1).w+neq_temp.qd(i)*z0);%连杆的角速度
        link(i).wd=R*link(i-1).wd+R*cross(neq_temp.qd(i)*link(i-1).w,z0)+neq_temp.qdd(i)*z0;%连杆的角加速度
        link(i).wd;
        a_temp1_=R*cross(link(i-1).wd,P);
        a_temp2_=R*(cross(link(i-1).w,cross(link(i-1).w,P)));
        a_temp3_=R*(link(i-1).a);
        link(i).a=a_temp1_+a_temp2_+a_temp3_;%连杆坐标系原点的线加速度
    end
    if i==2
        link(i).a=robot.gravity;
    end
    link(i).ac=cross(link(i).wd,link(i).pc)+cross(link(i).w,cross(link(i).w,link(i).pc))+link(i).a;%连杆i的质心的线加速度
    link(i).F=link(i).m*link(i).ac;
    link(i).N=link(i).I*link(i).wd+cross(link(i).wd,link(i).I*link(i).w);
    link(i).zm=[0;0;1];%电机转子旋转轴的方向
    link(i).wmd=link(motor_position(i)).wd+link(i).G*neq_temp.qdd(i)*link(i).zm+link(i).G*neq_temp.qd(i)*cross(link(motor_position(i)).w,link(i).zm); %电机转子i的角加速度 电机转子在连杆motor_position(i)上
%     link(i).wmd=link(i).G*neq_temp.qdd(i)*link(i).zm+link(i).G*neq_temp.qd(i)*cross(link(i-1).w,link(i).zm); %电机转子i相对于i-1的角加速度 电机转子在nmi连杆上   rne函数的算法，忽略了 link(motor_position(i)).wd 
end
fe=[0;0;0];
ne=[0;0;0];
%计算运动学参数R和P
for i=link_num:-1:2
    if i==link_num %计算运动学参数
        RE=[1 0 0;0 1 0;0 0 1];
        PE=[0;0;0];
    else
        R=T(1:3,1:3,i)'*T(1:3,1:3,i+1);% i+1在i中的旋转矩阵
        P=T(1:3,1:3,i)'*(T(1:3,4,i+1)-T(1:3,4,i));% ；i+1坐标系原点在i坐标系中的位置
    end
end
%计算连杆关节的力矩和关节的力矩
for i=link_num:-1:2
    if i==link_num %计算运动学参数
        RE=[1 0 0;0 1 0;0 0 1];
        PE=[0;0;0];
    else
        R=T(1:3,1:3,i)'*T(1:3,1:3,i+1);% i+1在i中的旋转矩阵
        P=T(1:3,1:3,i)'*(T(1:3,4,i+1)-T(1:3,4,i));% ；i+1坐标系原点在i坐标系中的位置
    end
    %计算关节处的力矩
    if i==link_num %若为末端杆 计算关节i处的力和力矩
        link(i).f=R*fe+link(i).F;
        link(i).nm=[0;0;0];%末端杆上没有电机转子，故力矩为0
        link(i).nj=link(i).N+RE'*ne+cross(link(i).pc,link(i).F)+cross(PE,R*fe);
        link(i).njm=link(i).nj+link(i).nm;%末端关节处的力矩
    else%若不是末端杆 计算关节i处的力和力矩
        link(i).f=R*link(i+1).f+link(i).F;%计算施加到关节上的力
        link(i).nm=(link(motor_position(i)).G*neq_temp.qdd(motor_position(i)))*link(motor_position(i)).Im*link(motor_position(i)).zm+...
                link(motor_position(i)).G*neq_temp.qd(motor_position(i))*link(motor_position(i)).Im*cross(link(i).w,link(motor_position(i)).zm);     %在连杆i的安装的电机转子转动造成连杆i的力矩附加量
            link(i).nj=link(i).N+R*link(i+1).njm+cross(link(i).pc,link(i).F)+cross(P,R*link(i+1).f);%连杆的惯性力和连杆惯性力矩对关节i处的造成的力矩值
%             link(i).njm=link(i).nj+link(i).nm;%关节i处的力矩
            link(i).njm=link(i).nj;%rne函数的关节i处的力矩算法，其忽略了link.nm
    end 
    %计算电机上的力矩
    if robot.links(1,i).sigma==1 %若为移动关节
        link(i).tau=link(i).f'*z0;
        link(i).mt=link(i).G*link(i).Im(3,3)*(link(i).wmd)'*link(i).zm;%电机i的惯量力矩引起的力矩附加值;
        link(i).taum=link(i).f'*z0+link(i).mt;
    else %若为旋转关节
        link(i).tau=link(i).njm'*z0; %不考虑电机的惯量力矩
        link(i).mt=link(i).G*link(i).Im(3,3)*(link(i).wmd)'*link(i).zm;%电机i的惯量力矩引起的力矩附加值
        link(i).taum=link(i).njm'*z0+link(i).mt;%驱动连杆i的电机所需的力矩
    end
    rne_tau(i)=link(i).taum;
end
end
% 不考虑电机转子
function [rne_tau,link]=cal_rne_nom(robot,neq,neqd,neqdd)
neq_temp.q=neq;%关节角度，相对于上一个连杆
neq_temp.qd=neqd;%关节角速度，相对于上一个连杆
neq_temp.qdd=neqdd;%关节角加速度，相对于上一个连杆
%link(1)表示机械臂的基座
link.w=zeros(3,1);%连杆角速度，相对于地球坐标系
link.wd=zeros(3,1);%连杆角加速度，相对于地球坐标系
link.p=zeros(3,1);%连杆坐标系原点的位置，连杆坐标系
% link.a=robot.gravity;
link.a=zeros(3,1);%连杆坐标系原点的线加速度，地球坐标系
link.ac=zeros(3,1);%连杆重心的线加速度，在地球坐标系
link.f=zeros(3,1);%连杆坐标系原点的受力，相对于连杆坐标系
link.tau=0;%连杆坐标系原点的力矩
link.pc=zeros(3,1);%连杆的质心的位置，在连杆坐标系中
link.m=0;%连杆的质量
link.I=zeros(3,3);%连杆的惯量矩阵，相对于质心
link.F=zeros(3,1);%连杆的惯性力大小，连杆坐标系
link.N=zeros(3,1);%连杆的惯性力矩，连杆坐标系
link.nj=zeros(3,1);%关节处的力矩
z0=[0;0;1];%Z方向向量
link_num_temp=size(robot.links);%连杆数量
link_num=link_num_temp(2);%连杆数量
T(1:4,1:4,1)=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];%T矩阵
%计算T矩阵 根据DH参数不同和连杆的关节类型而不同
if robot.mdh==1 %如果是改进的DH参数
    for i=2:link_num
        if robot.links(1,i).sigma==1
            robot.links(1,i).d=neq_temp.q(i);
        else
            robot.links(1,i).theta=neq_temp.q(i);
        end
        T(1:4,1:4,i)=T(1:4,1:4,i-1)*[
            cos(robot.links(1,i).theta),-sin(robot.links(1,i).theta),0,robot.links(1,i).a;
            sin(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),cos(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),-sin(robot.links(1,i).alpha),-sin(robot.links(1,i).alpha)*robot.links(1,i).d;
            sin(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),cos(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),cos(robot.links(1,i).alpha),cos(robot.links(1,i).alpha)*robot.links(1,i).d;
            0,0,0,1];
        link(i).p=T(1:3,4,i);
    end
else%如果是标准的DH参数
    for i=2:link_num
        if robot.links(1,i).sigma==1
            robot.links(1,i).d=neq_temp.q(i);
        else
            robot.links(1,i).theta=neq_temp.q(i);
        end
        T(1:4,1:4,i)=T(1:4,1:4,i-1)*[
            cos(robot.links(1,i).theta),-sin(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),sin(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),robot.links(1,i).a*cos(robot.links(1,i).theta);
            sin(robot.links(1,i).theta),cos(robot.links(1,i).theta)*cos(robot.links(1,i).alpha),-cos(robot.links(1,i).theta)*sin(robot.links(1,i).alpha),robot.links(1,i).a*sin(robot.links(1,i).theta);
            0,sin(robot.links(1,i).alpha),cos(robot.links(1,i).alpha),robot.links(1,i).d;
            0,0,0,1];
        link(i).p=T(1:3,4,i);
    end
end
%求连杆坐标系原点的线速度、连杆坐标系原点的线加速度、连杆的角速度、连杆的角加速度、质心的线加速度、电机转子的角加速度
for i=2:1:link_num %从序号2开始，其实
    R=T(1:3,1:3,i)'*T(1:3,1:3,i-1);% i-1在i中的旋转矩阵 R 上i下i-1
    P=T(1:3,1:3,i-1)'*(T(1:3,4,i)-T(1:3,4,i-1));% ；i坐标系原点在i-1坐标系中的位置 P上i-1下i
    link(i).pc=robot.links(1,i).r';
    link(i).I=robot.links(1,i).I;
    link(i).m=robot.links(1,i).m;
    if robot.links(i).sigma==1  %若为移动关节
        link(i).w=R*link(i-1).w;%连杆的角速度
        link(i).wd=R*link(i-1).w;%连杆的角加速度
        a_temp1=R*(cross(link(i-1).wd,P)+cross(link(i-1).w,cross(link(i-1).w,P))+link(i-1).a);
        a_temp2=2*cross(link(i).w,(neq_temp.qd(i)*z0));
        a_temp3=neq_temp.qdd(i)*z0;
        link(i).a=a_temp1+a_temp2+a_temp3;%连杆坐标系原点的线速度
    else  %若为旋转关节
        link(i).w=R*(link(i-1).w+neq_temp.qd(i)*z0);%连杆的角速度
        link(i).wd=R*link(i-1).wd+R*cross(neq_temp.qd(i)*link(i-1).w,z0)+neq_temp.qdd(i)*z0;%连杆的角加速度
        a_temp1_=R*cross(link(i-1).wd,P);
        a_temp2_=R*(cross(link(i-1).w,cross(link(i-1).w,P)));
        a_temp3_=R*(link(i-1).a);
        link(i).a=a_temp1_+a_temp2_+a_temp3_;%连杆坐标系原点的线加速度
    end
    link(i).ac=cross(link(i).wd,link(i).pc)+cross(link(i).w,cross(link(i).w,link(i).pc))+link(i).a;%连杆i的质心的线加速度
    link(i).F=link(i).m*link(i).ac;
    link(i).N=link(i).I*link(i).wd+cross(link(i).wd,link(i).I*link(i).w);
end
fe=[0;0;0];
ne=[0;0;0];
%计算运动学参数R和P
for i=link_num:-1:2
    if i==link_num %计算运动学参数
        RE=[1 0 0;0 1 0;0 0 1];
        PE=[0;0;0];
    else
        R=T(1:3,1:3,i)'*T(1:3,1:3,i+1);% i+1在i中的旋转矩阵
        P=T(1:3,1:3,i)'*(T(1:3,4,i+1)-T(1:3,4,i));% ；i+1坐标系原点在i坐标系中的位置
    end
end
%计算连杆关节的力矩和关节的力矩
for i=link_num:-1:2
    if i==link_num %计算运动学参数
        RE=[1 0 0;0 1 0;0 0 1];
        PE=[0;0;0];
    else
        R=T(1:3,1:3,i)'*T(1:3,1:3,i+1);% i+1在i中的旋转矩阵
        P=T(1:3,1:3,i)'*(T(1:3,4,i+1)-T(1:3,4,i));% ；i+1坐标系原点在i坐标系中的位置
    end
    %计算关节处的力矩
    if i==link_num %若为末端杆 计算关节i处的力和力矩
        link(i).f=R*fe+link(i).F;
        link(i).nj=link(i).N+RE'*ne+cross(link(i).pc,link(i).F)+cross(PE,R*fe);%不考虑关节处的力矩
    else%若不是末端杆 计算关节i处的力和力矩
        link(i).f=R*link(i+1).f+link(i).F;%计算施加到关节上的力
        link(i).nj=link(i).N+R*link(i+1).nj+cross(link(i).pc,link(i).F)+cross(P,R*link(i+1).f);%不考虑电机转子转动的关节i处的力矩
    end
    if robot.links(1,i).sigma==1 %若为移动关节
        link(i).tau=link(i).f'*z0;
    else %若为旋转关节
        link(i).tau=link(i).nj'*z0; %不考虑电机的惯量力矩
    end
    rne_tau(i)=link(i).tau;
end
end






