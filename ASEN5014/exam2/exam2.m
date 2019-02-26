close all; clear all; clc;
A=[-0.0869 0 0.039 -1; -4.424 -1.184 0 0.335; 0 1 0 0; 2.148 -.021 0 -.228];
B=[.0223;.547;0;-1.169];

% 4a
damp(A)

% b
trial=linspace(-1,0,1000);
ratio=zeros(length(trial),1);
count=1;
for i=trial
    kr=i;
    K=[0 0 0 kr];
    CL=A-B*K;
    [Wn,zeta,P]=damp(CL);
    ratio(count)=zeta(1);
    count=count+1;
end

[dis,ind]=min(abs(ratio-0.3));
trial(ind)

% c
K=[0 0 0 trial(ind)];
s = tf('s');
G = inv(s*eye(4)-A)*B;
Gdamp= inv(s*eye(4)-(A-B*K))*B;
figure
bode(G(4),Gdamp(4),{1e-2,1e2})
legend('Open Loop','Closed Loop')

% d
figure
step(G(4),Gdamp(4))
legend('Open Loop','Closed Loop')

%5c
a=0.01;
Kwash=[0 0 0 -.58];
Awash=[A-B*a*Kwash B*a;a a a a*Kwash(4) -a];
Bwash=[B;0];
Gwash_start=inv(s*eye(5)-Awash)*Bwash;

figure
bode(Gdamp(4),Gwash_start(4),{1e-2,1e2})
legend('4(c) damped','5(c) washout')

% d
count=1;
trial=linspace(-150,0,10000);
ratio=zeros(length(trial),1);
for i=trial
    Kwash=[0 0 0 i];
    Awash=[A-B*a*Kwash B*a;a*Kwash -a];
    [Wn,zeta,P]=damp(Awash);
    ratio(count)=zeta(1);
    count=count+1;
end
[dis,ind]=min(abs(ratio-0.3));
trial(ind)
Kwash=[0 0 0 trial(ind)];
Awash=[A-B*a*Kwash B*a;a a a a*Kwash(4) -a];
Bwash=[B;0];
Gwash=inv(s*eye(5)-Awash)*Bwash;
figure
bode(Gwash_start(4),Gwash(4),{1e-2,1e2});
legend('washout','washout damped system')

figure
step(G(4),Gdamp(4),Gwash(4))
legend('Open Loop','Closed Loop','Washout')