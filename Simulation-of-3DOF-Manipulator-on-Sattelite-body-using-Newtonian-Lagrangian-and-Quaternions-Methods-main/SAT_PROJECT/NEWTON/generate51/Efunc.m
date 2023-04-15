function E = Efunc(Ixx,Ixy,Ixz,Iyy,Iyz,Izz,L1,L2,L3,Ms,c,dq1,dq2,dq3,dq4,dq5,dq6,dq7,dq8,dq9,m1,m2,m3,q4,q5,q6,q7,q8,q9)
%EFUNC
%    E = EFUNC(IXX,IXY,IXZ,IYY,IYZ,IZZ,L1,L2,L3,MS,C,DQ1,DQ2,DQ3,DQ4,DQ5,DQ6,DQ7,DQ8,DQ9,M1,M2,M3,Q4,Q5,Q6,Q7,Q8,Q9)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    28-Jan-2020 21:48:13

t2 = cos(q4);
t3 = cos(q6);
t4 = sin(q4);
t5 = sin(q5);
t6 = sin(q6);
t8 = cos(q5);
t9 = dq4.*t5;
t10 = dq6-t9;
t12 = c./2.0;
t17 = t3.*t4;
t18 = t2.*t5.*t6;
t19 = t17-t18;
t20 = dq1.*t19;
t21 = t2.*t3;
t22 = t4.*t5.*t6;
t23 = t21+t22;
t24 = dq2.*t23;
t26 = dq3.*t6.*t8;
t75 = L1./2.0;
t76 = t12+t75;
t7 = t20-t24-t26+t10.*t76;
t11 = cos(q7);
t13 = dq5.*t3;
t14 = dq4.*t6.*t8;
t15 = t13+t14;
t16 = sin(q7);
t25 = L1+t12;
t27 = sin(q8);
t28 = t4.*t6;
t29 = t2.*t3.*t5;
t30 = t28+t29;
t31 = dq1.*t30;
t32 = t2.*t6;
t61 = t3.*t4.*t5;
t33 = t32-t61;
t34 = dq3.*t3.*t8;
t62 = dq2.*t33;
t35 = t31+t34-t62;
t36 = cos(q8);
t37 = t15.*t25;
t38 = dq1.*t2.*t8;
t39 = dq2.*t4.*t8;
t57 = dq3.*t5;
t40 = t37+t38+t39-t57;
t41 = t11.*t40;
t42 = t10.*t25;
t43 = t20-t24-t26+t42;
t66 = t16.*t43;
t44 = t41-t66;
t46 = cos(q9);
t55 = sin(q9);
t59 = t11.*t15;
t60 = t10.*t16;
t63 = t35.*t36;
t64 = dq8+t59-t60;
t65 = L2.*t64;
t67 = t27.*t44;
t68 = t63-t65+t67;
t69 = t27.*t35;
t70 = t36.*t44;
t71 = t69-t70;
t45 = t46.*t71+t55.*t68;
t47 = dq4.*t3.*t8;
t56 = dq5.*t6;
t48 = dq7+t47-t56;
t49 = t36.*t48;
t50 = t10.*t11;
t51 = t15.*t16;
t52 = t50+t51;
t53 = t27.*t52;
t54 = t49+t53;
t78 = t16.*t40;
t79 = t11.*t43;
t85 = t27.*t48;
t86 = t36.*t52;
t87 = t85-t86;
t90 = t46.*t54;
t91 = t55.*t87;
t92 = t90-t91;
t58 = -t78-t79+L2.*t54+(L3.*t92)./2.0;
t83 = dq8+dq9+t59-t60;
t72 = (L3.*t83)./2.0-t46.*t68+t55.*t71;
t73 = t47-t56;
t74 = t63+t67-(L2.*t64)./2.0;
t77 = t38+t39-t57+t15.*t76;
t80 = t78+t79-(L2.*t54)./2.0;
t81 = dq8./2.0;
t82 = (t11.*t15)./2.0;
t84 = L2.^2;
t88 = (dq5.*t6)./2.0;
t89 = L3.^2;
t93 = L1.^2;
E = -((dq5.*t3)./2.0+(dq4.*t6.*t8)./2.0).*(Ixy.*t10-Iyy.*t15+Iyz.*t73)-(dq6./2.0-(dq4.*t5)./2.0).*(-Ixx.*t10+Ixy.*t15+Ixz.*t73)+(Ms.*dq1.^2)./2.0+(Ms.*dq2.^2)./2.0+(Ms.*dq3.^2)./2.0+(t88-(dq4.*t3.*t8)./2.0).*(Ixz.*t10+Iyz.*t15-Izz.*t73)+(m1.*t7.^2)./2.0+(m1.*t35.^2)./2.0+(m3.*t45.^2)./2.0+(m3.*t58.^2)./2.0+(m2.*t71.^2)./2.0+(m3.*t72.^2)./2.0+(m2.*t74.^2)./2.0+(m1.*t77.^2)./2.0+(m2.*t80.^2)./2.0+(m1.*t93.*(t82-(t10.*t16)./2.0).*(t59-t60))./1.2e1+(m3.*t83.*t89.*(dq9./2.0+t81+t82-(t10.*t16)./2.0))./1.2e1+(m3.*t89.*(t54.*t55+t46.*t87).*((t54.*t55)./2.0+(t46.*t87)./2.0))./6.0e1+(m2.*t64.*t84.*(t81+t82-(t10.*t16)./2.0))./1.2e1+(m1.*t48.*t93.*(dq7./2.0-t88+(dq4.*t3.*t8)./2.0))./6.0e1+(m1.*t52.*t93.*((t10.*t11)./2.0+(t15.*t16)./2.0))./1.2e1+(m2.*t54.*t84.*((t27.*t52)./2.0+(t36.*t48)./2.0))./1.2e1+(m2.*t84.*t87.*((t27.*t48)./2.0-(t36.*t52)./2.0))./6.0e1+(m3.*t89.*t92.*((t46.*t54)./2.0-(t55.*t87)./2.0))./1.2e1;
