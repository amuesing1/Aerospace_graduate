function dxdt = ode_prob4lin(t,x,u)

%constants
K=1000;
G=6.673e-11;
M=5.98e24;
R=6.37e6;
g=50;
x3=1000;
u0=(G*M*x3)/(K*R^2);

% state
dxdt=[x(2);
    (2*G*M*x(1))/R^3-(g*x(2))/x3-(G*M*x(3))/(R^2*x3)+(K*(u(t)-u0))/x3; 
    -(u(t)-u0)];

end