function res = character_L(current_q, area_num, speed)
global L_time_flag
global L_time
L_time = 0;
%q_end = Move_2_area(current_q, area_num, speed);  %先移动到对应单元格上方
speed = 0.06;
current_q = current_q;
next_p = [100;  -28; 88; 20; 300; 10];
L_time_flag = 0;
q_end = current_2_next(current_q, next_p,0.15, 0); %Put down and prepare to write

current_q = q_end;
next_p = [100;  -28; 6; 20; 300; 10];
L_time_flag = 1;
pause(3)     
q_end = current_2_next(current_q,next_p,speed, 1);  % The first line |
L_time_flag = 0;

current_q = q_end;
next_p = [100;  -78; 7; 20; 300; 10];
L_time_flag = 1;
q_end = current_2_next(current_q,next_p,speed, 1);
L_time_flag = 0;

pause(3)     
res = character_O(current_q, area_num, 0.15);
end