function end_point = current_2_next(current_p, next_p, speed,  record)

% q_initial  = current_q;
% p_target =   next_p;
current_p = 200*current_p;  % Change the scale from meter to mm and divided by 5
x_dis = abs(current_p(1)-next_p(1));
y_dis = abs(current_p(2)-next_p(2));
z_dis = abs(current_p(3)-next_p(3));

steps = max(max(x_dis,y_dis),z_dis);
x = linspace(current_p(1), next_p(1),steps);
y = linspace(current_p(2), next_p(2),steps);
z = linspace(current_p(3), next_p(3),steps);

for i=1 :steps                              % 
       end_point = Locus(x(i),y(i),z(i), 1, 0); % Locus[x,y,z,~,~] x,y,z are target position
end

% 
% if  record
%     %tic
%     q_end = Jacobian_trajectory(q_initial, p_target, speed,  record);
%     Draw_UR5(q_end(1), q_end(2), q_end(3), q_end(4), q_end(5), q_end(6), 0)
% 
% 
% else
%     [~, ~, T_end] = Forward(current_q);
%     x_last = T_end(1,4)*200;
%     y_last = T_end(2,4)*200;
%     z_last = T_end(3,4)*200;
% 
%     mid_point_x = (x_last + next_p(1))/2 - 20;
%     mid_point_y = (y_last + next_p(2))/2;
%     mid_point_z = (z_last + next_p(3))/2;
%     p_target = [mid_point_x; mid_point_y; mid_point_z; next_p(4); next_p(5); next_p(6) ];
% 
%     q_end = Jacobian_trajectory(q_initial, p_target, speed,  record);
% 
%     q_initial = q_end;
% 
%     q_end = Jacobian_trajectory(q_initial, next_p, speed,  record);
%     
%     Draw_UR5(q_end(1), q_end(2), q_end(3), q_end(4), q_end(5), q_end(6), 0);
% 
% end

end








