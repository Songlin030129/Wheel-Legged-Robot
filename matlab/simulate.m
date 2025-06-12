
   
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

% 选择一个腿长进行仿真，例如取中间值
leg_length_sim = 0.1;
R1=0.0603;
L1=leg_length_sim/2;
LM1=leg_length_sim/2;
l1=0.01;
mw1=0.6;
mp1=0.38;
M1=3.3;
Iw1=0.5*mw1*R1^2;
Ip1=mp1*((L1+LM1)^2+0.48^2)/12.0;
IM1=M1*(0.135^2+0.066^2)/12.0;

% 重新计算A、B、K
syms x(t) T R Iw mw M L LM theta(t) l phi(t) mp g Tp Ip IM
syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

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

Q=diag([1 0.07 50 50 2000 0.6]);
R=[5 0;0 1];
K=lqr(A,B,Q,R);

% 状态空间仿真
x0 = [0.1; 0; 0; 0; 0.05; 0]; % 初始状态
sys = @(t, x) (A - B*K)*x;
tspan = [0 5];
[t, x] = ode45(sys, tspan, x0);

figure;
plot(t, x, 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('状态量');
legend('\theta','d\theta','x','dx','\phi','d\phi');
title(['LQR控制下系统状态响应仿真，腿长 = ' num2str(leg_length_sim) ' m']);
xlim([0 3]); % 限制横坐标范围
grid on;