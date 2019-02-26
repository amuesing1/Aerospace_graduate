clear all; close all; clc;

% Problem 1
m=1; %kg
l=1; %m
M=2; %kg
g=9.81; %m/s^2
Ahat=[0 1 0 0 0; 0 0 (m*g)/M 0 1/M; 0 0 0 1 0; 0 0 g/l+(m*g)/(M*l) 0 1/(M*l);...
    0 0 0 0 0];

dt=.05;

FGHM=expm(Ahat*dt);
F=FGHM(1:4,1:4);
abs(eig(F));
H=[1 0 -l 0];
O=[H*F; H*F^2; H*F^3; H*F^4];
rank(O);

load('midterm1problem1data.mat')
k1=Kct(1);
k2=Kct(2);
k3=Kct(3);
k4=Kct(4);

AhatCL=[0 1 0 0 0; -k1/M -k2/M (m*g-k3)/M -k4/M 0; 0 0 0 1 0;...
    -k1/(M*l) -k2/(M*l) g/l+(m*g-k3)/(m*l) -k4/(M*l) 0;0 0 0 0 0];
dt=.05;

FGHMCL=expm(AhatCL*dt);

F=FGHMCL(1:4,1:4);
abs(eig(F));
H=[1 0 -l 0];
O=[H*F; H*F^2; H*F^3; H*F^4];
rank(O);

x0=inv(O'*O)*O'*yNLhist(1:4)

x=zeros(length(yNLhist),4);
y=zeros(length(yNLhist),1);
x(1,:)=x0;
count=1;
for i=thist
    x(count+1,:)=F*x(count,:)';
    y(count,:)=H*x(count,:)';
    count=count+1;
end

figure(1)
hold on
plot(thist,y)
plot(thist,yNLhist)
legend('y(k)','YNLhist')
xlabel('k')
ylabel('Horizontal Displacement (m)')
title('CL DT LTI output response')

figure(2)
hold on
plot(thist,x(2:end,:))
legend('z','\delta z','\theta','\delta \theta')
xlabel('k')
title('CL DT LTI state response')

clear all;

% AQ1
