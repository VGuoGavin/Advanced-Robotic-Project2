    theta1=atan2(y,x); 
    % theta1=acos(x/sqrt(x*x+y*y));
    c=sqrt(x*x+y*y); % 末端到原点的距离
    theta3=acos((c*c+a*a-b*b)/(2*a*c));
    theta2=theta1-theta3; % 关节1 角度
    phi=pi-acos((a*a+b*b-c*c)/(2*a*b)); %关节2角度

