% 设一被控对象G（s）=50/(s^2+8s),
% 用增量式PID控制算法，增量式输出，编写仿真程序（输入分别为单位阶跃、正弦信号，
% 采样时间为10ms
% 仿真曲线包括系统输出及误差曲线，并加上注释、图例。程序如下
%
% 时间2016-05-06    作者：吕有才

clear all; close all; 
kc=1;h=0.1; Ti=0.5; Td=0.8; N=10;
a=(Td/N)/(h+Td/N);
b=kc*Td/(h+Td/N);
sys=tf(50,[1,8, 0]);
dsys=c2d(sys,h,'z');   %离散化
[num,den]=tfdata(dsys,'v');  
u_1=0.0;u_2=0.0; y_1=0.0;y_2=0.0; x=[0,0,0]'; error_1=0; error_2=0;
du_1=0;du_2=0;du_p=0; du_i=0;du_d=0;
u_p_1=0; 
u_i_1=0;u_i_2=0
u_d_1=kc*N;u_d_2=0;
for k=1:1:150 
    time(k)=k*h;   
    rin(k)=1
    %Linear model   
    du(k)=du_p+du_i+du_d;   %PID增量
    u(k)=u_2+du(k);         %增量式PID
    u_2=u_1;              %保存上一周期的PID输出
    u_1=u(k); 
    u_p(k)=u_p_1;
    u_i(k)=u_i_1;
    u_d(k)=u_d_1;
    du_2=du_1;              %更新上一采样的增量
    du_1=du(k);
%     u(k)=u_p_1+u_i_1+u_d_1;

%      yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*du_1+num(3)*du_2;  %增量输出
    % yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;  %位置输出
    error(k)=1; 

%     y_2=y_1;
%     y_1=yout(k);    
    error_2=error_1;     %保存上一周期的误差
    error_1=error(k); 
    du_p=kc*(error_1-error_2);    %比例部分增量
    du_i=kc*h*error_1/Ti;         %积分部分增量
    du_d=u_d_1-u_d_2;             %积分部分增量
    u_p_1=kc*error(k);
    u_i_2=u_i_1+kc*h/Ti*error(k);%u_i_2是下一采样值，u_i_1 为当前值
    u_i_1=u_i_2;
    u_d_2=u_d_1;
    u_d_1=a*u_d_2;%-b*(y_1-y_2);
end
figure(1);  
% plot(time,du,'b',time,u,'r');  
plot(time,u); 
xlabel('time(s)');
ylabel('rin,yout');  
% figure(2);  
% plot(time,error,'r')  
% xlabel('time(s)');
% ylabel('error');