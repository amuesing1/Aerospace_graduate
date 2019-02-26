close all; clear all; clc;

% 10.6
M=[2 1 0 0;1 4 1 0;0 1 4 1;0 0 1 2];
K=[1 -1 0 0;-1 2 -1 0;0 -1 2 -1;0 0 -1 1];
MK=inv(M)*K;
eigenvalues=eig(MK);
freq=sqrt(eigenvalues);

Mswap=[2 1 0 0; 1 4 1 0; 0 1 4 1;0 0 1 2];
Kswap=[1 -1 0 0; -1 2 -1 0; 0 -1 2 -1; 0 0 -1 1];
Kbb=Kswap(1:2,1:2);
Kbi=Kswap(3:4,1:2);
Kib=Kswap(1:2,3:4);
Kii=Kswap(3:4,3:4);

Kbb_cond=Kbb-Kbi*inv(Kii)*Kib;

T=[eye(2);-inv(Kii)*Kib];
Mhat=T'*M*T;
Khat=T'*K*T;
MK_guyan=inv(Mhat)*Khat;
eigenvalues_g=eig(MK_guyan);
freq_guyan=sqrt(eigenvalues_g);

% 10.7
T=[1 1/6 1/2 -1/6 1/2]';
K=[2 -1 -1 1 -1; -1 4 0 -2 0;-1 0 2 0 0;1 -2 0 4 0;-1 0 0 0 2];
Keq=T'*K*T;