function dxdt = ode_prob4(t,x,u)

%constants
K=1000;
G=6.673e-11;
M=5.98e24;
R=6.37e6;
g=50;

% state
dxdt=[x(2);(K*u(t)-g*x(2))/x(3)-(G*M)/(R+x(1))^2; -u(t)];

end