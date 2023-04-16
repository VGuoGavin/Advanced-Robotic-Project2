%% 旋转矩阵
function R=Rot(theta,ch)
% @brief: 绕某个轴的旋转矩阵的求法
% @param: theta,绕ch轴旋转的角度；ch,x、y、z中的某个轴
% @ret: 绕 ch 轴的旋转矩阵
% @birth: created by MY on 20200218
c=cos(theta);
s=sin(theta);
switch(ch)
    case'x'
        R=[1,0,0;0,c,-s;0,s,c];
    case'y'
        R=[c,0,s;0,1,0;-s,0,c];
    case'z'
        R=[c,-s,0;s,c,0;0,0,1];
end

