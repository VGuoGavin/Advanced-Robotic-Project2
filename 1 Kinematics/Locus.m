function end_point = Locus(x,y,z, type, write) %x,y,z, type:Delete last frame or not

% global X2_g; % used for storing the points
% global Y2_g;
% global Z2_g;

[~, ~, T_end] = Forward([0, 0,0]); % First get Transforme matrix
T = T_end;
T (1,4) = x/200;         % exchange unit = mm * 1000 / 5
T (2,4) = y/200;
T (3,4) = z/200;
Inv = real(Backward(T));  % Inverse 

% Use the inv res to calculate the target postion again
[~, ~, T_trajectory]= Forward([Inv(1,1),Inv(1,2),Inv(1,3)]);  
%disp(Inv);

if write    % draw the path or not
    Draw_UR(Inv(1,1),Inv(1,2),Inv(1,3), type);
    X2_g = [X2_g, T_trajectory(1, 4)*200];    % Store the path points
    Y2_g = [Y2_g, T_trajectory(2, 4)*200];
    Z2_g = [Z2_g, T_trajectory(3, 4)*200];
    end_point = T_trajectory(1:3, 4);
else
    Draw_UR(Inv(1,1),Inv(1,2),Inv(1,3), type);
    end_point = T_trajectory(1:3, 4);
    text(200*T_trajectory(1, 4), 200*T_trajectory(2, 4),200* T_trajectory(3, 4), '('+string(200*T_trajectory(1, 4))+','+string(200*T_trajectory(2, 4))+','+string(200*T_trajectory(3, 4))+')');
end

end