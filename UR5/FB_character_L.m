function res = FB_character_L(Current_p, area_num, speed)

p_end = FB_Move_2_area(Current_p, 2, 1);  %先移动到对应单元格上方

speed = 1;
current_p = p_end;
next_p = [100;  -28; 88];
p_end = FB_current_2_next(current_p, next_p,speed, 0); %Put down and prepare to write

current_p = p_end;
next_p = [100;  -28; 6];
p_end = FB_current_2_next(current_p,next_p,speed, 1);  % The first line |


current_p = p_end;
next_p = [100;  -78; 7];
p_end = FB_current_2_next(current_p,next_p,speed, 0.9);

FB_character_O(p_end, area_num, 1.4)
end