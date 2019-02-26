clear all; clc;
A=[0 1;-1 0];
expm(A);
eAt=[cos(1) sin(1);-sin(1) cos(1)];
M=eye(2);
M*eAt*inv(M);

Ahat=[0 1 0;-1 0 1; 0 0 0];
expm(Ahat);

A=[-2 1;2 -3];
[vec,val]=eig(A);
M=[1 -1;1 2];
val=eig(M'*M);
val=eig(M);
sqrt(val(2))/sqrt(val(1));
norm(M)*norm(inv(M));
cond(M);
norm([1 0]);

A=[-1 0 0;0 -2 0;0 -2 -1];
[vec,val]=eig(A);
M=[1 0 0; 0 0 1; 0 1 2];
cond(M);
norm([2;-1]);
1/(2.2361*5.8284);

A=[0 1; -2 -2];
[vec,val]=eig(A);
expm(A);
eAt=exp(-1).*[cos(1) sin(1);-sin(1) cos(1)];
M=[-1 -1;2 0];
M*eAt*inv(M);

Ahat=[0 1 0;-2 -2 1; 0 0 0];
expm(Ahat);
cond(M)