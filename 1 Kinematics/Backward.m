function theta=Backward(T)
% DH table for the Project
% Robot dimensions (sample values)
d1 = 0.0892;
a2 = 0.425;
a3 = 0.392;

% Joint variables (these will be calculated)
theta1 = 0;
theta2 = 0;
theta3 = 0;

DH = [ theta1,  d1,     0,  pi/2;
           theta2,  0,     a2,  0;
           theta3,  0,     a3,  0];
       
% Final homogeneous transformation matrix (sample values)
% These sample values were found using a forward kinematics calculation
% Extract some known values from the DH table
d1 = double(DH(1,2));
a2 = double(DH(2,3));
a3 = double(DH(3,3));

% Solve for theta2&3
xyz_bar = double(T*[0;0;0;1]);
x_bar = xyz_bar(1);
y_bar = xyz_bar(2);
z_bar = xyz_bar(3);
L = sqrt(x_bar^2 + y_bar^2);

% Solve for theta2
beta = atan2((z_bar-d1),(L));
gamma = acos((L^2+(z_bar-d1)^2+a2^2-a3^2)/(2*a2*sqrt((L)^2+(z_bar-d1)^2)));
theta2 = gamma+beta;

% Solve for theta3
theta3 = fsolve(@(theta)[cos(theta); sin(theta)]-(1/(a3^2))*[a3*cos(theta2), a3*sin(theta2); -a3*sin(theta2), a3*cos(theta2)]*[L-a2*cos(theta2);z_bar-a2*sin(theta2)-d1],0);

% Solve for theta1
theta1 = fsolve(@(theta) [cos(theta); sin(theta)]-(1/(x_bar^2+y_bar^2))*[x_bar, y_bar; y_bar, -x_bar]*[a3*cos(theta3)*cos(theta2)+a2*cos(theta2)-a3*sin(theta2)*sin(theta3); 0], 0);
theta = [real(theta1), real(theta2),real(theta3)];

end

