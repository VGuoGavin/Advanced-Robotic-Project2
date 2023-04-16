function q_end = Move_2_area(current_q, area_num, speed)
% Jacobian method trajectory control

q_initial  = current_q;
if area_num == 1   % first quadrant
    p_target = [80;  50; 50; 20; 250; 10];
    q_end = Jacobian_trajectory(q_initial, p_target, speed, 0);
    Draw_UR5(q_end(1), q_end(2), q_end(3), q_end(4), q_end(5), q_end(6), 0);
else
    if area_num == 2  % second quadrant
        p_target = [80;  -50; 50; 20; 250; 10];
        q_end = Jacobian_trajectory(q_initial, p_target, speed, 0);
        Draw_UR5(q_end(1), q_end(2), q_end(3), q_end(4), q_end(5), q_end(6), 0);
    else 
        if area_num == 3  % third quadrant
            p_target = [80;  50; -50; 20; 250; 10];
            q_end = Jacobian_trajectory(q_initial, p_target, speed, 0);
            Draw_UR5(q_end(1), q_end(2), q_end(3), q_end(4), q_end(5), q_end(6), 0);
        else  % forth quadrant
            p_target = [80;  -50; -50; 20; 250; 10];
            q_end = Jacobian_trajectory(q_initial, p_target, speed, 0);
            Draw_UR5(q_end(1), q_end(2), q_end(3), q_end(4), q_end(5), q_end(6), 0);
        end
    end
end
