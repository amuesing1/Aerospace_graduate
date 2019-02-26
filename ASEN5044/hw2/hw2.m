% Jeremy Muesing
% ASEN 5044 HW2

close all; clear all; clc;

%% Problem 1

k=398600; %km^3/s^2
r0=6678; %km
w0=sqrt(k/(r0^3)); %angular velocity (rad/s)

Ahat=[0 1 0 0 0 0;w0^2+(2*k)/(r0^3) 0 0 2*r0*w0 1 0; 0 0 0 1 0 0;...
    0 -2*w0/r0 0 0 0 1/r0; 0 0 0 0 0 0; 0 0 0 0 0 0];
dt=10;

FGHM=expm(Ahat*dt);

%% Problem 1.19

% initial conditions
time=[0,5];
x3=1000;
x0=[0; 0 ; x3];
% just checking perturbations
x0lin=[0 ; 0; 0];
K=1000; %thrust constant
G=6.673e-11; %m^3/kg/s^2
M=5.98e24; %kg
R=6.37e6; %m
u0=(G*M*x3)/(K*R^2);
du=10;
u=@(t) u0+du*abs(cos(t));

[t1,x1]=ode45(@(t,x) ode_prob4(t,x,u),time,x0);
[t2,x2]=ode45(@(t,x) ode_prob4lin(t,x,u),time,x0lin);

figure(1)
ax1=subplot(3,1,1);
hold(ax1,'on');
plot(t1,x1(:,1))
plot(t2,x2(:,1))
grid
title('\Delta u=10')
xlabel('Time (s)')
ylabel('Position (m)')
legend('Non-linear','linear')

du=100;
u=@(t) u0+du*abs(cos(t));

[t1,x1]=ode45(@(t,x) ode_prob4(t,x,u),time,x0);
[t2,x2]=ode45(@(t,x) ode_prob4lin(t,x,u),time,x0lin);

ax2=subplot(3,1,2);
hold(ax2,'on');
plot(t1,x1(:,1))
plot(t2,x2(:,1))
grid
title('\Delta u=100')
xlabel('Time (s)')
ylabel('Position (m)')
legend('Non-linear','linear')

du=300;
u=@(t) u0+du*abs(cos(t));

[t1,x1]=ode45(@(t,x) ode_prob4(t,x,u),time,x0);
[t2,x2]=ode45(@(t,x) ode_prob4lin(t,x,u),time,x0lin);

ax3=subplot(3,1,3);
hold(ax3,'on');
plot(t1,x1(:,1))
plot(t2,x2(:,1))
grid
title('\Delta u=300')
xlabel('Time (s)')
ylabel('Position (m)')
legend('Non-linear','linear')