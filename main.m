clc
clear
robot_model;%机器人建模
q=[0 0 1 1 1];%某一瞬时的关节的角度
qd=[0 1 1 2 1];%某一瞬时的关节的角速度
qdd=[0 0 2 1 1];%某一瞬时的关节的角加速度
motor_position=[1 1 1 1 1];%若包含参数motor_position，表示考虑电机转子惯量，驱动连杆i的电机安装在第motor_position（i）个连杆上
%taum：计算得到电机的力矩；
%L：结构体，包含计算过程中的连杆的各种参数，具体看m文件里
[taum,L]=cal_rne(ZRobot,q,qd,qdd,motor_position);%考虑电机转子惯量
%taum：计算得到电机的力矩；startup_rtb
%L：结构体，包含计算过程中的连杆的各种参数，具体看m文件
% [taum,L]=cal_rne(ZRobot,q,qd,qdd); %不考虑电机转子惯量
taum
ZRobot.rne(q,qd,qdd)

