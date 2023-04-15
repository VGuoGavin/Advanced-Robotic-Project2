function [ J ] = Jacobiano( robot )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

J = sym(zeros(6,6));
T_endeffector = DKin (robot);
pe = T_endeffector(1:3,4);
z0 = sym([0;0;1]);
Jp = cross(z0,pe);
Jo = z0;

J(1:3,1) = Jp;
J(4:6,1) = Jo;

for i=2:6
    T = DKin(robot(1:i-1,:));
    z = T(1:3,3);
    p = T(1:3,4);
    
    Jp = cross(z,pe-p);
    Jo = z;
    
    J(1:3,i) = Jp;
    J(4:6,i) = Jo;
end
end

