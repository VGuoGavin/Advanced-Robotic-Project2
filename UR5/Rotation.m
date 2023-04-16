function [R] = Rotation(r, ax)

    R = (eye(4));
    rot = ([cos(r),-sin(r);sin(r),cos(r)]);

    if (ax == 'x')
        R(2:3,2:3) = rot;
    elseif (ax == 'z')
        R(1:2,1:2) = rot;
    end

end