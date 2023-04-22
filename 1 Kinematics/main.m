close all
clear all
%  Can run these two parts separately
warning off
%% Forwards and inverse part
for i=1 :1 :100                              % 
       end_point = Locus(i,100-i,100-i, 1, 0); % Locus[x,y,z,~,~] x,y,z are target position
end

next_p = [163, 0, 17.8];                                     % Target position
current_2_next(end_point, next_p, 0.1, 0)   % Move from one point to another one

%% Jacobian part
q_begin = [0, 0, 0];             % Initial torque [theta1, theta2, theta3]
p_target = [50, 50, 50];     % target position[x,y,z]
speed = 0.1;                            % Here set joints rotate in the same speed
record = 1;                             % Draw the trajectory or not
Jacobian_trajectory(q_begin, p_target, speed, 1); 