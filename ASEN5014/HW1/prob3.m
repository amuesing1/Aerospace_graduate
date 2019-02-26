% Code to evaluate the behavior of a mass-spring-damper-system for 20sec
% with (a) no forced input and no damping and (b) both damping and forced
% input.

close all; clear all; clc;

%% part a

% initial conditions
time=0:0.0001:20;
x0=[1; 2];
u=@(t) 0*t;
c=0;

[t,x]=ode23(@(t,x) ode_prob3(t,x,u,c),time,x0);

% plotting
figure(1)
subplot(2,2,1)
plot(t,x(:,1))
grid
title('Position vs Time')
xlabel('Time (s)')
ylabel('Position (m)')

subplot(2,2,2)
plot(t,0.5*1*x(:,2).^2)
grid
title('Kentic Energy vs Time')
xlabel('Time (s)')
ylabel('Kinetic Energy (J)')

subplot(2,2,3)
plot(t,4*x(:,1).^3)
grid
title('Force of the Nonlinear Spring vs Time')
xlabel('Time (s)')
ylabel('Force (N)')

subplot(2,2,4)
plot(x(:,1),x(:,2))
grid
title('Postion vs Velocity')
xlabel('Position (m)')
ylabel('Velocity (m/s)')

%% part b
u=@(t) sin(8*t);
c=0.2;

[t,x]=ode23(@(t,x) ode_prob3(t,x,u,c),time,x0);

% plotting
figure(2)
subplot(2,2,1)
plot(t,x(:,1))
grid
title('Position vs Time')
xlabel('Time (s)')
ylabel('Position (m)')

subplot(2,2,2)
plot(t,0.5*1*x(:,2).^2)
grid
title('Kentic Energy vs Time')
xlabel('Time (s)')
ylabel('Kinetic Energy (J)')

subplot(2,2,3)
plot(t,4*x(:,1).^3)
grid
title('Force of the Nonlinear Spring vs Time')
xlabel('Time (s)')
ylabel('Force (N)')

subplot(2,2,4)
plot(x(:,1),x(:,2))
grid
title('Postion vs Velocity')
xlabel('Position (m)')
ylabel('Velocity (m/s)')