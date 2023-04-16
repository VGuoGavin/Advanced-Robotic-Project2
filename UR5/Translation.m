

function [T] = Translation(d, ax)
    T = (eye(4));
    if (ax == 'x')
        T(1,4) =+ d;
    elseif (ax == 'z')
        T(3,4) =+ d;
    end
end
