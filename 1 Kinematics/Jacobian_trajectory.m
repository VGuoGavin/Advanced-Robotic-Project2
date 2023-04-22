function q_end  = Jacobian_trajectory(q_begin, p_target, speed,  record)

global X_g;         %Used for storing the trajectory points
global Y_g;
global Z_g;

XX_real = [];  
YY_real = [];  
ZZ_real = [];  

%Unit translation because of the size of image : The calculation = 200 : 1
p_target = p_target/200;   
q = q_begin;
q_comf = q;
eps = 0.17 ;
C = eye(6) * 1000;
step = 0;

Winv = zeros(6);
for i = 1:6
    Winv(i,i) = i/6 + 0.1;
end

[~, ~, T_end, ~]  = Jacobian_calculation(q(1), q(2), q(3)); 
T_full_curr = T_end;            % Get the destination first
y(1:3) = T_full_curr(1:3,4);
y(4) = atan2(T_full_curr(3,2), T_full_curr(3,3));
y(5) = atan2(-T_full_curr(3,1), sqrt(T_full_curr(3,2)^2 + T_full_curr(3,3)^2));
y(6) = atan2(T_full_curr(2,1), T_full_curr(1,1));
%error = sum(abs(p_target - y));     % calculate the error distance to determine stop or not
distance = sqrt(   (p_target(1) - y(1))^2+ (p_target(2) - y(2))^2+ (p_target(3) - y(3))^2  );

while (distance > eps)
        [~, ~, T_end, J_curr] = Jacobian_calculation(q(1),  q(2), q(3));
        % Get the end position and Jocabian matrix
        Jh =  pinv(J_curr);   % Winv * J_curr' * inv(J_curr * Winv * J_curr' + inv(C));  % Calculate the inverse of Jacobian matrix
        y = zeros(6,1);
        T_full_curr = T_end; 
        y(1:3) = T_full_curr(1:3,4);
        y(4) = atan2(T_full_curr(3,2), T_full_curr(3,3));
	    y(5) = atan2(-T_full_curr(3,1), sqrt(T_full_curr(3,2)^2 + T_full_curr(3,3)^2));
	    y(6) = atan2(T_full_curr(2,1), T_full_curr(1,1));
        distance = sqrt(   (p_target(1) - y(1))^2+ (p_target(2) - y(2))^2+ (p_target(3) - y(3))^2  );
        q = q + speed * Jh*(p_target - y); 
        Draw_UR(q(1), q(2), q(3), 1);
        if step >0 &&  record % Control draw trajectory or not
            XX_real(step) = 200*T_full_curr(1,4);
            YY_real(step) = 200*T_full_curr(2,4);
            ZZ_real(step) = 200*T_full_curr(3,4);
            plot3(XX_real, YY_real, ZZ_real, 'r.');  % draw trajectory
            hold on;
        end
        step = step + 1;    % Count the steps
end
Draw_UR(q(1), q(2), q(3), 0);
X_g = [X_g, XX_real];
Y_g = [Y_g, YY_real];
Z_g = [Z_g, ZZ_real];
q_end = q;
end
