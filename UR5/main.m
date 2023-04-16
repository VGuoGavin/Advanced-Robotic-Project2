close all
clear all
global Link

%% Forwards and Backwards part

num = 1;
speed = [0.5,1,0.5,0,0,0];
Current_p = [10, 10, 100];
area_num = 1;
FB_character_F(Current_p, area_num, speed)
% need comment the last several lines codes in "Draw_UR5.m" to show the trajectory


%% Jacobian part
    
close all
clear all
speed = 0.1;
current_q = [0, 0, 0];
area_num = 1;
q_end = Character_F(current_q, area_num, speed);
next_p = [0;  0; 165; 20; 300; 10];
q_end = current_2_next(q_end,next_p,0.12, 0); 
% need comment the last several lines code to show the trajectory
