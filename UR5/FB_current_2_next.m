function p_end = FB_current_2_next(Current_p, next_p, speed, write)


x_next = next_p(1);
y_next = next_p(2);
z_next = next_p(3);
write_or_not = write;
if write
    num = max([abs(Current_p(1) - x_next), abs(Current_p(2)-y_next), abs(Current_p(3)-z_next)])/2;
    
    X_target = linspace(Current_p(1), x_next, num*speed);
    Y_target = linspace(Current_p(2), y_next, num*speed);
    Z_target = linspace(Current_p(3), z_next, num*speed);
    
    for i = 1: num*speed
        Locus(X_target(i), Y_target(i), Z_target(i), 1, write_or_not);
        p_end = [X_target(i), Y_target(i), Z_target(i)];
    end

else
    
    mid_point_x = (Current_p(1) + x_next)/2 - 20;
    mid_point_y = (Current_p(2) + y_next)/2;
    mid_point_z = (Current_p(3) + z_next)/2;
    
    num = round(max([abs(mid_point_x - Current_p(1)), abs(mid_point_y-Current_p(2)), abs(mid_point_z-Current_p(3))])/2);
    X_up = linspace(Current_p(1), mid_point_x, num*speed);
    Y_up = linspace(Current_p(2), mid_point_y, num*speed);
    Z_up = linspace(Current_p(3), mid_point_z, num*speed);
    
    X_down = linspace(mid_point_x, x_next, num*speed);
    Y_down = linspace(mid_point_y, y_next, num*speed);
    Z_down = linspace(mid_point_z, z_next, num*speed);
    
    X_path = [X_up X_down];
    Y_path = [Y_up Y_down];
    Z_path = [Z_up Z_down];
    
    for i = 1: 2*num*speed
        Locus(X_path(i), Y_path(i), Z_path(i), 1, write_or_not);
    end
    p_end = [x_next, y_next, z_next];

end


end








