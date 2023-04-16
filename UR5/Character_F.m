
function res = Character_F(current_q, area_num, speed)
% To write "F" by using Jacobian method
global F_time_flag
global F_time

% First move to the first quadrant to prepare for writing
q_end = Move_2_area(current_q, area_num, 0.18);      
F_time_flag = 0
current_q = q_end;
next_p = [100;  72; 88; 20; 300; 10];
q_end = current_2_next(current_q, next_p,0.15, 0); % Put down on canvas

pause(3)                                %pause 3 seconds
current_q = q_end;
next_p = [100;  72; 6; 20; 300; 10];
F_time = 0;
F_time_flag = 1;
q_end = current_2_next(current_q,next_p, 0.079, 1); % The first line 
F_time_flag = 0;

current_q = q_end;
next_p = [100;  72; 88; 20; 300; 10];
q_end = current_2_next(current_q,next_p, 0.15, 0);  % Go back prepare for the second draw


current_q = q_end;
next_p = [100;  28; 88; 20; 300; 10];
F_time_flag = 1;
q_end = current_2_next(current_q,next_p, 0.08, 1); % Second part -
F_time_flag = 0;

current_q = q_end;
next_p = [100;  72; 56; 20; 300; 10];
q_end = current_2_next(current_q,next_p,0.15, 0); % Go to next draw beginning


current_q = q_end;
next_p = [100;  28; 56; 20; 300; 10];
F_time_flag = 1;
q_end = current_2_next(current_q,next_p,0.075, 1); % Third part -
F_time_flag = 0;
pause(3)     
res = character_L(q_end, 2, 1.4);                % Go to next character

end