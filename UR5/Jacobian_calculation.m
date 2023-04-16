
function [T_01, T_02, T_end, J] = Jacobian_calculation(q1, q2, q3)
% This function is used to calculate the J
% First to get the T
[T_01, T_02, T_end] = Forward([q1, q2, q3]);
p_eff = T_end(1:3,4);     % The end position
J = (zeros(6));
z0 = [0;0;1];
% To calculate the position from 0-6 one by one
J(1:3,1) = cross(z0, p_eff - z0);
J(1:3,2) = cross(T_01(1:3,3), p_eff - T_01(1:3,4));
J(1:3,3) = cross(T_02(1:3,3), p_eff - T_02(1:3,4));
% J(1:3,4) = cross(T_03(1:3,3), p_eff - T_03(1:3,4));
% J(1:3,5) = cross(T_04(1:3,3), p_eff - T_04(1:3,4));
% J(1:3,6) = cross(T_05(1:3,3), p_eff - T_05(1:3,4));
% To calculate the orienation from 0-6 one by one
J(4:6,1) = z0;
J(4:6,2) = T_01(1:3,3);
J(4:6,3) = T_02(1:3,3);
% J(4:6,4) = T_03(1:3,3);
% J(4:6,5) = T_04(1:3,3);
% J(4:6,6) = T_05(1:3,3);

end