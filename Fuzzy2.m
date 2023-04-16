%% 模糊PID控制文件
function [deltPID]=Fuzzy2(PID,realAng,fuzzTab)
 
valErr=LinearQuantization(PID,realAng);% 偏差线性化的值
[indexErr,valueErr]=CalcMemberShip(valErr(1),fuzzTab);%偏差的隶属度计算
[indexDErr,valueDErr]=CalcMemberShip(valErr(2),fuzzTab);%偏差的变化量的隶属度计算
FuzVal(1)=valueErr(1)*(valueDErr(1)*PID.pTab(indexErr(1),indexDErr(1))+valueDErr(2)*PID.pTab(indexErr(1),indexDErr(2)))...
          +valueErr(2)*(valueDErr(1)*PID.pTab(indexErr(2),indexDErr(1))+valueDErr(2)*PID.pTab(indexErr(2),indexDErr(2)));
FuzVal(2)=valueErr(1)*(valueDErr(1)*PID.iTab(indexErr(1),indexDErr(1))+valueDErr(2)*PID.iTab(indexErr(1),indexDErr(2)))...
          +valueErr(2)*(valueDErr(1)*PID.iTab(indexErr(2),indexDErr(1))+valueDErr(2)*PID.iTab(indexErr(2),indexDErr(2)));
FuzVal(3)=valueErr(1)*(valueDErr(1)*PID.dTab(indexErr(1),indexDErr(1))+valueDErr(2)*PID.dTab(indexErr(1),indexDErr(2)))...
          +valueErr(2)*(valueDErr(1)*PID.dTab(indexErr(2),indexDErr(1))+valueDErr(2)*PID.dTab(indexErr(2),indexDErr(2)));
 
deltPID=LinearDeb(PID,FuzVal);
 
function retW=LinearQuantization(PID,realAng)
%=======   线性化函数 【-6，6】
% retW(1)：偏差的线性化
% retW(1)：偏差导数的线性化
error=PID.ref-realAng; 
derror=error-PID.err; 
retW(1)=6*error/(PID.max-PID.min);
retW(2)=3*derror/(PID.max-PID.min);
 
 
function [indexMem,valueMem]=CalcMemberShip(err,fuzzTab)
% 隶属度计算函数
% fuzzTab 模糊规则表
% fuzzTab=[-6 -4 -2  0  2  4  6]
%         [NB NM NS ZO PS PM PB]
 
NB=fuzzTab(1);
NM=fuzzTab(2); 
NS=fuzzTab(3);
Z0=fuzzTab(4); 
PS=fuzzTab(5); 
PM=fuzzTab(6); 
PB=fuzzTab(7);
indexMem=[];
valueMem=[];
if err>=NB && err<NM
    indexMem(1)=0;
    indexMem(2)=1;
    valueMem(1)=-0.5*err-2.0;  % y=0.5x-2
    valueMem(2)=0.5*err+3.0;   % y=0.5x+3
elseif err>=NM && err<NS
    indexMem(1)=1;
    indexMem(2)=2;
    valueMem(1)=-0.5*err-1.0;  % y=-0.5x-1
    valueMem(2)=0.5*err+2.0;   % y=0.5x+2
elseif err>=NS && err<Z0
    indexMem(1)=2;
    indexMem(2)=3;
    valueMem(1)=-0.5*err;  % y=-0.5x
    valueMem(2)=0.5*err+1.0; % y=0.5x+1
elseif err>=Z0 && err<PS
    indexMem(1)=3;
    indexMem(2)=4;
    valueMem(1)=-0.5*err+1.0;  % y=-0.5x+1
    valueMem(2)=0.5*err; % y=0.5x
elseif err>=PS && err<PM
    indexMem(1)=4;
    indexMem(2)=5;
    valueMem(1)=-0.5*err+2;  % y=-0.5x+2
    valueMem(2)=0.5*err-1.0; % y=0.5x-1
elseif err>=PM && err<=PB
    indexMem(1)=5;
    indexMem(2)=6;
    valueMem(1)=-0.5*err+3.0;  % y=-0.5x+3
    valueMem(2)=0.5*err-2.0; % y=0.5x+1-2
end
indexMem=indexMem+1;
 
 
function FuzVal=LinearDeb(PID,FuzVal)
% 限幅处理
if FuzVal(1)>PID.maxDltKp 
   FuzVal(1)=PID.maxDltKp ;
elseif FuzVal(1)<PID.minDltKp 
   FuzVal(1)=PID.minDltKp ;
end
if FuzVal(2)>PID.maxDltKi 
   FuzVal(2)=PID.maxDltKi ;
elseif FuzVal(2)<PID.minDltKi 
   FuzVal(2)=PID.minDltKi;
end
if FuzVal(3)>PID.maxDltKd 
   FuzVal(3)=PID.maxDltKd ;
elseif FuzVal(3)<PID.minDltKd 
   FuzVal(3)=PID.minDltKd;
end
 
 
 
 
 

