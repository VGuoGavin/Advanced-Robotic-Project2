function Connect3D(p1,p2,option,pt)        
%function that connects two joints into a bar, and Link(i).p represents the spatial position of the i-th joint
% Draw a straight line from point p1 to point p2. Both points p1 and p2 are matrixes with four rows and one column, 
% but the values ​​of the first three rows are taken here. option is the line color value.
h = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],option);    
set(h,'LineWidth',pt)    % Here pt is the line width, which is the width of the robot rod.

