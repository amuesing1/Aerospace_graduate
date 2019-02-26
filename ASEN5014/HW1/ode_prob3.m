function dxdt = ode_prob3(t,x,u,c)

%constants
m=1;
k=4;

% state
dxdt=[x(2);(-k/m)*x(1)-(c/m)*x(2)+(1/m)*u(t)-k*x(1)^3];

end

