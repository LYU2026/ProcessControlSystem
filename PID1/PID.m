% ��һ���ض���G��s��=50/(s^2+8s),
% ��λ��ʽPID�����㷨��д�����������Ϊ��λ��Ծ�źţ�
% ����ʱ��Ϊ10ms��
% �������߰���ϵͳ�����������ߣ�������ע�͡�ͼ��
%
% ʱ��2016-05-04    ���ߣ����в�

clear all; close all; 
kc=1;h=0.01; Ti=5; Td=0.8; N=10;
a=(Td/N)/(h+Td/N);
b=kc*Td/(h+Td/N);
sys=tf(50,[1,8, 0]);
dsys=c2d(sys,h,'z'); 
[num,den]=tfdata(dsys,'v');  
u_1=0.0;u_2=0.0; y_1=0.0;y_2=0.0; x=[0,0,0]'; error_1=0; error_2=0;
u_p_1=0; 
u_i_1=0;u_i_2=0
u_d_1=0;u_d_2=0;
for k=1:1:2000 
    time(k)=k*h;   
    rin(k)=1     %��Ծ�ź�
    %Linear model  
    u(k)=u_p_1+u_i_1+u_d_1;  %PID���
    yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;  %ϵͳ���
    error(k)=rin(k)-yout(k);  %�������
    u_2=u_1;
    u_1=u(k);  
    y_2=y_1;
    y_1=yout(k);     
    error_2=error_1; 
    error_1=error(k); 
    u_p_1=kc*error(k);    %��������
    u_i_2=u_i_1+kc*h/Ti*error(k);%u_i_2����һ����ֵ��u_i_1 Ϊ��ǰֵ
    u_i_1=u_i_2;
    u_d_2=u_d_1;
    u_d_1=a*u_d_2-b*(y_1-y_2); %΢�ֲ���
end
figure(1);  
plot(time,rin,'b',time,yout,'r');  
xlabel('time(s)');
ylabel('rin,yout');  
figure(2);  
plot(time,error,'r')  
xlabel('time(s)');
ylabel('error');