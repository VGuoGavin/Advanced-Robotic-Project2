function [v1, v2, v3, v4, v5, v6] = invKin(Robot)

% mp=matrix de posição | mr=matriz de rotação
syms q1 q2 q3 q4 q5 q6 real
mp = sym('mp',[3 1]);
mr = sym('mr',3);
T_end (1:3,1:3) = mr;
T_end (1:3,4) = mp;
T_end(4,1:4) = [0 0 0 1];
syms sh el wr real

%resolver em ordem a q1. 0.0823 e d6 e 0.10915 d4.
a1 = (mp(2)-Robot(6,1)*mr(2,3));
a2 = (mp(1)-Robot(6,1)*mr(1,3));
b = Robot(4,1)/(sqrt(((mp(1)-Robot(6,1)*mr(1,3))^2)+((mp(2)-Robot(6,1)*mr(2,3))^2)));
%impor condiçao shoulder up/down
v1 = atan2(a1,a2)+ sh*acos(b)+(pi/2);
%q1 = solve((823*mr2_3*cos(q1))/10000 - mp2*cos(q1) - (823*mr1_3*sin(q1))/10000 + mp1*sin(q1)==2183/20000);
%q1 = simplify (q1);

%resolver em ordem a q5. Impor condiçao wrist up/down
v5 = wr*(acos((mp(1)*sin(v1)-mp(2)*cos(v1)-Robot(4,1))/Robot(6,1)));
%q5 = solve((mp1*sin(q1))/(cos(q1)^2 + sin(q1)^2) - (mp2*cos(q1))/(cos(q1)^2 + sin(q1)^2)==(823*cos(q5))/10000 + 2183/20000);
%q5 = simplify (q5);

%resolver em ordem a q6
sinq6 = (-mr(1,2)*sin(v1)+mr(2,2)*cos(v1))/(sin(v5));
cosq6 = (-mr(2,1)*cos(v1)+mr(1,1)*sin(v1))/(sin(v5));
v6 = atan2(sinq6,cosq6);
%q6 = simplify (q6);

%resolver e retirar q3
%0.09456 é o valor de a5
%pwx = mp(1)*cos(q1) - (33*mr(1,3)*cos(q1))/400 - (33*mr(2,3)*sin(q1))/400 + mp(2)*sin(q1) - (109*mr(1,3)*cos(q1)*cos(q5))/1000 - (591*mr(1,2)*cos(q1)*cos(q6))/6250 - (109*mr(2,3)*cos(q5)*sin(q1))/1000 - (591*mr(1,1)*cos(q1)*sin(q6))/6250 - (591*mr(2,2)*cos(q6)*sin(q1))/6250 - (591*mr(2,1)*sin(q1)*sin(q6))/6250 + (109*mr(1,1)*cos(q1)*cos(q6)*sin(q5))/1000 + (109*mr(2,1)*cos(q6)*sin(q1)*sin(q5))/1000 - (109*mr(1,2)*cos(q1)*sin(q5)*sin(q6))/1000 - (109*mr(2,2)*sin(q1)*sin(q5)*sin(q6))/1000;
%pwy = mp(3) - (33*mr(3,3))/400 - (109*mr(3,3)*cos(q5))/1000 - (591*mr(3,2)*cos(q6))/6250 - (591*mr(3,1)*sin(q6))/6250 + (109*mr(3,1)*cos(q6)*sin(q5))/1000 - (109*mr(3,2)*sin(q5)*sin(q6))/1000;
Trans = inv(DKin(Robot(1,:)))*T_end*inv(DKin(Robot(4:6,:)));
pwx = Trans(1,4);
pwy = Trans(2,4);
pwx = subs (pwx, [q1 q5 q6], [v1 v5 v6]);
pwy = subs (pwy, [q1 q5 q6], [v1 v5 v6]);
cosq3 = ((pwx^2)+(pwy^2)-((Robot(2,3))^2)-((Robot(3,3))^2))/(2*(Robot(2,3))*Robot(3,3));
%impor condiçao para elbow up/down
sinq3 = el*sqrt(1-(cosq3^2));
v3 = atan2(sinq3,cosq3);
%q3 = simplify(q3);

sinq2 = ((Robot(2,3)+Robot(3,3)*cosq3)*pwy - Robot(3,3)*sinq3*pwx)/((pwx^2)+(pwy^2));
cosq2 = ((Robot(2,3)+Robot(3,3)*cosq3)*pwx + Robot(3,3)*sinq3*pwy)/((pwx^2)+(pwy^2));
v2 = -atan2 (cosq2, sinq2);
%q2 = simplify(q2);


cosss = -mr(3,2)*cos(v6) - mr(3,1)*sin(v6);
sennn = mr(1,2)*cos(v1)*cos(v6) + mr(1,1)*cos(v1)*sin(v6) + mr(2,2)*cos(v6)*sin(v1) + mr(2,1)*sin(v1)*sin(v6);
angulo = atan2(sennn,cosss);
v4 = angulo - v3 - v2;

% cosq4 = mr(3,1)*sin(q2 + q3)*cos(q5)*cos(q6) - mr(1,3)*cos(q2 + q3)*cos(q1)*sin(q5) - mr(3,3)*sin(q2 + q3)*sin(q5) - mr(2,3)*cos(q2 + q3)*sin(q1)*sin(q5) - mr(3,2)*sin(q2 + q3)*cos(q5)*sin(q6) + mr(2,1)*cos(q2 + q3)*cos(q5)*cos(q6)*sin(q1) - mr(1,2)*cos(q2 + q3)*cos(q1)*cos(q5)*sin(q6) - mr(2,2)*cos(q2 + q3)*cos(q5)*sin(q1)*sin(q6) + mr(1,1)*cos(q2 + q3)*cos(q1)*cos(q5)*cos(q6);
% sinq4 = mr(3,1)*cos(q2 + q3)*cos(q5)*cos(q6) - mr(3,3)*cos(q2 + q3)*sin(q5) - mr(3,2)*cos(q2 + q3)*cos(q5)*sin(q6) + mr(1,3)*sin(q2 + q3)*cos(q1)*sin(q5) + mr(2,3)*sin(q2 + q3)*sin(q1)*sin(q5) - mr(1,1)*sin(q2 + q3)*cos(q1)*cos(q5)*cos(q6) - mr(2,1)*sin(q2 + q3)*cos(q5)*cos(q6)*sin(q1) + mr(1,2)*sin(q2 + q3)*cos(q1)*cos(q5)*sin(q6) + mr(2,2)*sin(q2 + q3)*cos(q5)*sin(q1)*sin(q6);
% q4 = atan2(sinq4,cosq4);
%q4 = simplify(q4);
