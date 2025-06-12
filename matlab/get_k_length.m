
   
    %theta : 摆杆与竖直方向夹角             R   ：驱动轮半径
    %x     : 驱动轮位移                    L   : 摆杆重心到驱动轮轴距离
    %phi   : 机体与水平夹角                LM  : 摆杆重心到其转轴距离
    %T     ：动轮输出力矩驱                 l   : 机体重心到其转轴距离
    %Tp    : 髋关节输出力矩                 mw  : 驱动轮转子质量
    %N     ：驱动轮对摆杆力的水平分量        mp  : 摆杆质量
    %P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量
    %Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量
    %Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量
    %Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量
clc;
clear
n=1;
for leg_length = 0.1:0.01:0.3
syms x(t) T R Iw mw M L LM theta(t) l phi(t) mp g Tp Ip IM
syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

R1=0.0603;                        %驱动轮半径
L1=leg_length/2;                  %摆杆重心到驱动轮轴距离
LM1=leg_length/2;                 %摆杆重心到其转轴距离
l1=0.01;                          %机体质心距离转轴距离
mw1=0.6;                          %驱动轮质量
mp1=0.38;                          %杆质量
M1=3.3;                             %机体质量
Iw1=0.5*mw1*R1^2;                 %驱动轮转动惯量
Ip1=mp1*((L1+LM1)^2+0.48^2)/12.0; %摆杆转动惯量
IM1=M1*(0.135^2+0.066^2)/12.0;   %机体绕质心转动惯量


NM = M*diff(x + (L + LM )*sin(theta)-l*sin(phi),t,2);
N = NM + mp*diff(x + L*sin(theta),t,2);
PM = M*g + M*diff((L+LM)*cos(theta)+l*cos(phi),t,2);
P = PM +mp*g+mp*diff(L*cos(theta),t,2);

eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
eqn2 = Ip*diff(theta,t,2) == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
eqn3 = IM*diff(phi,t,2) == Tp +NM*l*cos(phi)+PM*l*sin(phi);

eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);

[f1,f2,f3] = solve(eqn10,eqn20,eqn30,f1,f2,f3);

A=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[theta0,d_theta,x0,d_x,phi0,d_phi]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
A=subs(A,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
A=double(A);
B=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[T,Tp]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
B=subs(B,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
B=double(B);

% %theta d_theta x d_x phi d_phi
% Q=diag([1 0.07 10 5 300 0.6]); 
% %T Tp
% R=[35 0;0 1];      

%theta d_theta x d_x phi d_phi
Q=diag([1 0.07 50 50 2000 0.6]); 
%T Tp
R=[5 0;0 1];       

K=lqr(A,B,Q,R);

a11 = zeros(12,6);
a(1,n) = K(1,1);
a(2,n) = K(1,2);  
a(3,n) = K(1,3);
a(4,n) = K(1,4); 
a(5,n) = K(1,5);
a(6,n) = K(1,6);  
a(7,n) = K(2,1);
a(8,n) = K(2,2);
a(9,n) = K(2,3);
a(10,n)= K(2,4);  
a(11,n)= K(2,5);
a(12,n)= K(2,6); 
n = n+1;
end

leg_x = 0.1:0.01:0.3;
for i=1:21
y11(1,i) = a(1,i); 
y12(1,i) = a(2,i);
y13(1,i) = a(3,i); 
y14(1,i) = a(4,i);
y15(1,i) = a(5,i); 
y16(1,i) = a(6,i);
y21(1,i) = a(7,i); 
y22(1,i) = a(8,i);
y23(1,i) = a(9,i); 
y24(1,i) = a(10,i);
y25(1,i) = a(11,i); 
y26(1,i) = a(12,i); 
end
%函数拟合
p11 = polyfit(leg_x,y11,3);
p12 = polyfit(leg_x,y12,3);
p13 = polyfit(leg_x,y13,3);
p14 = polyfit(leg_x,y14,3);
p15 = polyfit(leg_x,y15,3);
p16 = polyfit(leg_x,y16,3);
p21 = polyfit(leg_x,y21,3);
p22 = polyfit(leg_x,y22,3);
p23 = polyfit(leg_x,y23,3);
p24 = polyfit(leg_x,y24,3);
p25 = polyfit(leg_x,y25,3);
p26 = polyfit(leg_x,y26,3);

fprintf('float Polynomial_Coefficient[12][4] = { \n');
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p11(1), p11(2), p11(3), p11(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p12(1), p12(2), p12(3), p12(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p13(1), p13(2), p13(3), p13(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p14(1), p14(2), p14(3), p14(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p15(1), p15(2), p15(3), p15(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p16(1), p16(2), p16(3), p16(4));

fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p21(1), p21(2), p21(3), p21(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p22(1), p22(2), p22(3), p22(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p23(1), p23(2), p23(3), p23(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p24(1), p24(2), p24(3), p24(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff},\n', p25(1), p25(2), p25(3), p25(4));
fprintf('{%9.3ff,%9.3ff,%9.3ff,%9.3ff} };\n', p26(1), p26(2), p26(3), p26(4));


% %函数反拟合绘制
% q11 = polyval(p11,leg_x);
% q12 = polyval(p12,leg_x);
% q13 = polyval(p13,leg_x);
% q14 = polyval(p14,leg_x);
% q15 = polyval(p15,leg_x);
% q16 = polyval(p16,leg_x);
% q21 = polyval(p21,leg_x);
% q22 = polyval(p22,leg_x);
% q23 = polyval(p23,leg_x);
% q24 = polyval(p24,leg_x);
% q25 = polyval(p25,leg_x);
% q26 = polyval(p26,leg_x);   
% %函数拟合图绘制
% subplot(2,6,1);
% plot(leg_x,y11,'black');
% hold on;
% plot(leg_x,q11,'g--');
%         title('K11')
% subplot(2,6,2);
% plot(leg_x,y12,'black');
% hold on;
% plot(leg_x,q12,'g--');
%         title('K12')
% subplot(2,6,3);
% plot(leg_x,y13,'black');
% hold on;
% plot(leg_x,q13,'g--');
%         title('K13')
% subplot(2,6,4);
% plot(leg_x,y14,'black');
% hold on;
% plot(leg_x,q14,'g--');
%         title('K14')
% subplot(2,6,5);
% plot(leg_x,y15,'black');
% hold on;
% plot(leg_x,q15,'g--');
%         title('K15')
% subplot(2,6,6);
% plot(leg_x,y16,'black');
% hold on;
% plot(leg_x,q16,'g--');
%         title('K16')
% subplot(2,6,7);
% plot(leg_x,y21,'black');
% hold on;
% plot(leg_x,q21,'g--');
%         title('K21')
% subplot(2,6,8);
% plot(leg_x,y22,'black');
% hold on;
% plot(leg_x,q22,'g--');
%         title('K22')
% subplot(2,6,9);
% plot(leg_x,y23,'black');
% hold on;
% plot(leg_x,q23,'g--');
%         title('K23')
% subplot(2,6,10);
% plot(leg_x,y24,'black');
% hold on;
% plot(leg_x,q24,'g--');
%         title('K24')
% subplot(2,6,11);
% plot(leg_x,y25,'black');
% hold on;
% plot(leg_x,q25,'g--');
%         title('K25')
% subplot(2,6,12);
% plot(leg_x,y26,'black');
% hold on;
% plot(leg_x,q26,'g--');
%         title('K26')

