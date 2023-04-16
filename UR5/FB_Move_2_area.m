function p_end = FB_Move_2_area(Current_p, area_num, speed)
% Forwards and Backwards method trajectory control
if area_num == 1
    x_next = 80;
    y_next = 50;
    z_next = 50;
    num = max([abs(Current_p(1) - x_next), abs(Current_p(2)-y_next), abs(Current_p(3)-z_next)])/2;
    X_target = linspace(Current_p(1), x_next, num*speed);
    Y_target = linspace(Current_p(2), y_next, num*speed);
    Z_target = linspace(Current_p(3), z_next, num*speed);
    write_or_not = 0;
    for i = 1: num*speed
        Locus(X_target(i), Y_target(i), Z_target(i), 1, write_or_not);
    end
    p_end = [x_next, y_next, z_next];
else
    if area_num == 2
    x_next = 80;
    y_next = -50;
    z_next = 50;
    num = max([abs(Current_p(1) - x_next), abs(Current_p(2)-y_next), abs(Current_p(3)-z_next)])/2;
    X_target = linspace(Current_p(1), x_next, num*speed);
    Y_target = linspace(Current_p(2), y_next, num*speed);
    Z_target = linspace(Current_p(3), z_next, num*speed);
    write_or_not = 0;
    for i = 1: num*speed
        Locus(X_target(i), Y_target(i), Z_target(i), 1, write_or_not);
    end
    p_end = [x_next, y_next, z_next];
    else 
        if area_num == 3
            x_next = 80;
            y_next = 50;
            z_next = -50;
            num = max([abs(Current_p(1) - x_next), abs(Current_p(2)-y_next), abs(Current_p(3)-z_next)])/2;
            X_target = linspace(Current_p(1), x_next, num*speed);
            Y_target = linspace(Current_p(2), y_next, num*speed);
            Z_target = linspace(Current_p(3), z_next, num*speed);
            write_or_not = 0;
            for i = 1: num*speed
                Locus(X_target(i), Y_target(i), Z_target(i), 1, write_or_not);
            end
            p_end = [x_next, y_next, z_next];
        else

            x_next = 80;
            y_next = -50;
            z_next = -50;
            num = max([abs(Current_p(1) - x_next), abs(Current_p(2)-y_next), abs(Current_p(3)-z_next)])/2;
            X_target = linspace(Current_p(1), x_next, num*speed);
            Y_target = linspace(Current_p(2), y_next, num*speed);
            Z_target = linspace(Current_p(3), z_next, num*speed);
            write_or_not = 0;
            for i = 1: num*speed
                Locus(X_target(i), Y_target(i), Z_target(i), 1, write_or_not);
            end
            p_end = [x_next, y_next, z_next];
        end
    end
end
