function res = character_W(current_q, area_num, speed)
global W_time
global W_time_flag
W_time = 0;

%q_end = Move_2_area(current_q, 4,speed);  %先移动到对应单元格上方
speed = 0.138;
current_q = current_q;
next_p = [100;  -5; -20; 20; 300; 10];
q_end = current_2_next(current_q, next_p,0.12, 0); %Put down and prepare to write

W_time_flag = 1;
current_q = q_end;
next_p = [100;  -25; -85; 20; 300; 10];
pause(3)     
q_end = current_2_next(current_q,next_p,speed, 1);  % The first line |

current_q = q_end;
next_p = [100;  -50; -16; 20; 300; 10];
q_end = current_2_next(current_q,next_p,speed, 1);  % The first line |

current_q = q_end;
next_p = [100;  -75; -85; 20; 300; 10];
q_end = current_2_next(current_q,next_p,speed, 1);  % The first line |

current_q = q_end;
next_p = [100;  -95; -16; 20; 300; 10];
res = current_2_next(current_q,next_p,speed, 1);  % The first line |
pause(3)     
W_time_flag = 0;
end