function [T_01, T_02, T_end] = Forward(theta)
% DH table
   %           a      alfa          d         teta
DH=[        0,    pi/2,	  0.0892,	  theta(1);
         0.425,	     0,	  0.0,	      theta(2);
         0.392,	     0,	  0.0,         theta(3)];

% Calculate the T one by one
T_01 = (Translation(DH(1,3), 'z') * Rotation(DH(1,4), 'z') * Translation(DH(1,1), 'x') * Rotation(DH(1,2), 'x'));
T_12 = (Translation(DH(2,3), 'z') * Rotation(DH(2,4), 'z') * Translation(DH(2,1), 'x') * Rotation(DH(2,2), 'x'));
T_23 = (Translation(DH(3,3), 'z') * Rotation(DH(3,4), 'z') * Translation(DH(3,1), 'x') * Rotation(DH(3,2), 'x'));
 
%Calculate the T 0-6 
T_02 = (T_01 * T_12);
T_end = (T_02 * T_23);

end

