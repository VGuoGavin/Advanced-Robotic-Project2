function Robot = UR3()
%   Returns D-H table of parameters for Robotic Arm
%   Robot=[d v a alpha offset;
%          d v a alpha offset;
%          . . .   .   offset;
%          d v a alpha offset];
%   Use symbolic variables for each joint coordinate of the robot: in the d
%   column for a prismatic joint and in the v column for a rotational
%   joint. Name the variables from q1 to qn. In the last column, insert the
%   coordinate offset for the manipulator Home position.

syms q1 q2 q3 q4 q5 q6 real
Robot = [ 0         q1   0        pi/2    0;
          0         q2   0.244    0       pi/2;
          0         q3   0.213    0       0;
          0.110     q4   0        pi/2   pi/2;
          0.083     q5   0        -pi/2    0;
          0.082     q6   0        0       0];

end

