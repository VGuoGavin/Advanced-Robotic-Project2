%% Generate Matlab and Simulink Code for RobotX
%  Create Simulink Library file RobotX_Lib.mdl
new_system('UR3_Lib','Library');
open_system('UR3_Lib');

%% Direct Kinematics and Jacobian
[UR3_T]=DKin(UR3);
UR3_R=UR3_T(1:3,1:3);
UR3_p=UR3_T(1:3,4);
[v1, v2, v3, v4, v5, v6] = invKin(UR3);
[ J ] = Jacobiano( UR3 );
[G,C_dq,B] = NewtonEul();

%Generate optimized embeded Matlab function blocks for Simulink
matlabFunctionBlock('UR3_Lib/anthro_Direct_Kinematics',UR3_R,UR3_p, J);
matlabFunctionBlock('UR3_Lib/Inverse_Kinematics', v1,v2,v3,v4,v5,v6);
matlabFunctionBlock('UR3_Lib/Dynamics', G, C_dq, B);
%% Save library in current Directory
save_system('UR3_Lib');
close_system('UR3_Lib');
