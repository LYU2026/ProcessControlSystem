% 设一被控对象G（s）=50/(s^2+8s),
% 用位置式PID控制算法编写仿真程序，输入为单位阶跃信号，
% 采样时间为10ms，
% 仿真曲线包括系统输出及误差曲线，并加上注释、图例
%
% 时间2016-05-04    作者：吕有才

clear all; close all; 
kc=1;h=0.1; Ti=0.5; Td=0.8; N=10;
a=(Td/N)/(h+Td/N);
b=kc*Td/(h+Td/N);
sys=tf(50,[1,8, 0]);
dsys=c2d(sys,h,'z'); 
[num,den]=tfdata(dsys,'v');  
u_1=0.0;u_2=0.0; y_1=0.0;y_2=0.0; x=[0,0,0]'; error_1=0; error_2=0;
u_p_1=0; 
u_i_1=0;
u_d_1=N*kc;
for k=1:1:25
    time(k)=k*h;   
    rin(k)=1     %阶跃信号
    %Linear model  
      %PID输出
%     yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;  %系统输出
    error(k)=1;%rin(k)-yout(k);  %计算误差
    u_2=u_1;
%    u_1=u(k);  
%     y_2=y_1;
%     y_1=yout(k);     
    error_2=error_1; 
    error_1=error(k); 
    u_p_1=kc*error(k);    %比例部分
    u(k)=u_p_1+u_i_1+u_d_1;
    u_p(k)=u_p_1;
    u_i(k)=u_i_1;
    u_d(k)=u_d_1;
    u_i_2=u_i_1+kc*h/Ti*error(k);%u_i_2是下一采样值，u_i_1 为当前值
    u_i_1=u_i_2;
    u_d_2=u_d_1;
    u_d_1=a*u_d_2;%-b*(y_1-y_2); %微分部分 
end
figure(1);  
% plot(time,u_i,'black');  
plot(time,u_p,'b',time,u_i,'r',time,u_d,'green',time,u,'black');  
xlabel('time(s)');
ylabel('rin,yout');  
% figure(2);  
% plot(time,error,'r')  
% xlabel('time(s)');
% ylabel('error');