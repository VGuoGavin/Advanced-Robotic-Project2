function res = mid_point(x_last, y_last, z_last, x_next, y_next, z_next)

        mid_point_x = (x_last + x_next)/2 - 20;
        mid_point_y = (y_last + y_next)/2;
        mid_point_z = (z_last + z_next)/2;

        num = max([abs(mid_point_x - x_last), abs(mid_point_y-y_last), abs(mid_point_z-z_last)])/2;
        X_up = linspace(x_last, mid_point_x, num/2);
        Y_up = linspace(y_last, mid_point_y, num/2);
        Z_up = linspace(z_last, mid_point_z, num/2);

        X_down = linspace(mid_point_x, x_next, num/2);
        Y_down = linspace(mid_point_y, y_next, num/2);
        Z_down = linspace(mid_point_z, z_next, num/2);

        X_path = [X_up X_down];
        Y_path = [Y_up Y_down];
        Z_path = [Z_up Z_down];

        for i = 1: num
            Locus(X_path(i), Y_path(i), Z_path(i), 1);
        end
        res = [x_next, y_next, z_next];

end