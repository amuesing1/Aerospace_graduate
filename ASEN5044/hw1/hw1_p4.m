close all; clear all; clc;

Iy=750; %kgm^3
Iz=1000; %kgm^3
Ix=500; %kgm^3
p0=20; %rad/s
x0=[0;0.1;0];
t0=0;
tf=5;
dt=0.1;
x=zeros(3,tf/dt);


A=[0 0 0; 0 0 p0*(Ix-Iz)/Iy; 0 p0*(Iy-Ix)/Iz 0];

for t=t0:dt:tf
    x(:,round(t/dt+1))=expm(A*t)*x0;
end

t=0:0.1:5;
figure
hold on
grid
plot(t,x(1,:))
plot(t,x(2,:))
plot(t,x(3,:))
legend('\Delta p','\Delta q','\Delta r')
ylabel('Rad/s')
xlabel('Time (s)')
title('State Time History')