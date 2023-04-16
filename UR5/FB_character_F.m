function res = FB_character_F(Current_p, area_num, speed)
speed = 1;
p_end = FB_Move_2_area(Current_p, area_num, 1);  %先移动到对应单元格上方

current_p = p_end;
 
next_p = [100;  72; 88];
p_end = FB_current_2_next(current_p, next_p, 1, 0); %Put down and prepare to write

current_p = p_end;
next_p = [100;  72; 6];
p_end = FB_current_2_next(current_p, next_p, 1, 1);  % The first line |

current_p = p_end;
next_p = [100;  72; 88];
p_end = FB_current_2_next(current_p,next_p, speed, 0);  % Go back prepare for the second -

current_p = p_end;
next_p = [100;  28; 88];
p_end = FB_current_2_next(current_p,next_p,speed, 1);

current_p = p_end;
next_p = [100;  72; 56];
p_end = FB_current_2_next(current_p,next_p, speed, 0);

current_p = p_end;
next_p = [100;  28; 56];
p_end = FB_current_2_next(current_p,next_p, speed, 1);
FB_character_L(p_end, 2, speed)
end