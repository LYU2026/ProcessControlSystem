%Pt100
clear all;

R0=100;
A=3.9083E-3;
B=-5.775E-7;
C=-4.23225E-12;

t=-2000:0.1:5000;
RTD=R0*(1+A*t+B*t.^2+C*(t-100).*t.^3).*(t<=0)+R0*(1+A*t+B*t.^2).*(t>0);

plot(t,RTD);

xlabel('temperature (^oC)');
ylabel('^RRTD (ohm)');
title('Pt100');
