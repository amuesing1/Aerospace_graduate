close all; clear all; clc;

data=[-1 0; -.9 .1; -.5 .9; 0 1; .5 .85; .75 .5; 1 0; 0 -1.5];

A=[]*length(data);
y=[]*length(data);
for i=1:length(data)
    A(i,:)=[2*data(i,1) 2*data(i,2) 1];
    y(i)=data(i,1)^2+data(i,2)^2;
end
y=y';

x=A'*pinv(A*A')*y
r=sqrt(x(3)+x(1)^2+x(2)^2);

figure(1)
hold on
axis equal
grid on
th=0:pi/50:2*pi;
xpoints=r*cos(th)+x(1);
ypoints=r*sin(th)+x(2);
h1=plot(xpoints,ypoints);
h2=plot(x(1),x(2),'kO');
h3=scatter(data(:,1),data(:,2),'rx');
xlabel('x')
ylabel('y')
legend([h2 h3],'center','data points')