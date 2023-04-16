function res = FB_character_W(current_p, area_num, speed)

%p_end = FB_Move_2_area(current_p, 4,speed);  %先移动到对应单元格上方

speed = 1
current_p = current_p;
next_p = [100;  -5; -20; 20; 300; 10];
p_end = FB_current_2_next(current_p, next_p,0.12, 0); %Put down and prepare to write

current_p = p_end;
next_p = [100;  -25; -85; 20; 300; 10];
p_end = FB_current_2_next(current_p,next_p,speed, 1);  % The first line |

current_p = p_end;
next_p = [100;  -50; -16; 20; 300; 10];
p_end = FB_current_2_next(current_p,next_p,speed, 1);  % The first line |

current_p = p_end;
next_p = [100;  -75; -80; 20; 300; 10];
p_end = FB_current_2_next(current_p,next_p,speed, 1);  % The first line |

current_p = p_end;
next_p = [100;  -95; -16; 20; 300; 10];
p_end = FB_current_2_next(current_p,next_p,speed, 1);  % The first line |

current_p = p_end;
p_end = FB_current_2_next(current_p,[10, 0, 100], 2, 0);
Locus(p_end(1), p_end(2),p_end(3), 0, 0)
end