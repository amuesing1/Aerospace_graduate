close all; clear all; clc;

% Problem 1
A=[0 1;-100 -10];
gamma=[0;1];
W=10;
z=0.2*[-A gamma*W*gamma'; zeros(2) A'];
state_trans=expm(z);
F=state_trans(3:4,3:4)';
Q=F*state_trans(1:2,3:4);

% Problem 3
s=[66.6 84.9 88.6 78 96.8 105.2 93.2 111.6 88.3 117 115.2];
t=linspace(0,10,11);

n=2;
H=zeros(length(s),n);
R=eye(length(s));
for i=1:length(s)
    H(i,:)=[1 t(i)];
    R(i,i)=cov(s);
end
p1=inv(H'*inv(R)*H)*H'*inv(R)*s';

n=3;
H=zeros(length(s),n);
for i=1:length(s)
    H(i,:)=[1 t(i) t(i)^2];
end
p2=inv(H'*inv(R)*H)*H'*inv(R)*s';

n=4;
H=zeros(length(s),n);
for i=1:length(s)
    H(i,:)=[1 t(i) t(i)^2 t(i)^3];
end
p3=inv(H'*inv(R)*H)*H'*inv(R)*s';

n=5;
H=zeros(length(s),n);
for i=1:length(s)
    H(i,:)=[1 t(i) t(i)^2 t(i)^3 t(i)^4];
end
p4=inv(H'*inv(R)*H)*H'*inv(R)*s';

figure
hold on
scatter(t,s)
plot(t,p1(2)*t+p1(1));
xlabel('Years since 1946')
ylabel('Tons of Steel')
title('Linear Fit')

figure
hold on
scatter(t,s)
plot(t,p2(3)*t.^2+p2(2)*t+p2(1));
xlabel('Years since 1946')
ylabel('Tons of Steel')
title('Quadratic Fit')

figure
hold on
scatter(t,s)
plot(t,p3(4)*t.^3+p3(3)*t.^2+p3(2)*t+p3(1));
xlabel('Years since 1946')
ylabel('Tons of Steel')
title('Cubic Fit')

figure
hold on
scatter(t,s)
plot(t,p4(5)*t.^4+p4(4)*t.^3+p4(3)*t.^2+p4(2)*t+p4(1));
xlabel('Years since 1946')
ylabel('Tons of Steel')
title('Quartic Fit')

e1=sqrt(mean((p1(2)*t+p1(1)-s).^2));
e2=sqrt(mean((p2(3)*t.^2+p2(2)*t+p2(1)-s).^2));
e3=sqrt(mean((p3(4)*t.^3+p3(3)*t.^2+p3(2)*t+p3(1)-s).^2));
e4=sqrt(mean((p4(5)*t.^4+p4(4)*t.^3+p4(3)*t.^2+p4(2)*t+p4(1)-s).^2));

pred1=p1(2)*11+p1(1);
pred2=p2(3)*11.^2+p2(2)*11+p2(1);
pred3=p3(4)*11.^3+p3(3)*11.^2+p3(2)*11+p3(1);
pred4=p4(5)*11.^4+p4(4)*11.^3+p4(3)*11.^2+p4(2)*11+p4(1);

% Problem 4
% a
R=[8 5.15 6.5;5.15 5 -4.07;6.5 -4.07 50];
S=chol(R,'lower');
T=100;
GPS=zeros(T,3);
for i=1:T
    GPS(i,:)=[1;1;1]+S*randn(3,1);
end

figure
subplot(1,3,1)
scatter(GPS(:,1),GPS(:,2))
axis([-10 10 -20 20])
grid on
title('y_k(1) vs y_k(2)')
xlabel('y_k(1)')
ylabel('y_k(2)')

subplot(1,3,2)
scatter(GPS(:,1),GPS(:,3))
axis([-10 10 -20 20])
grid on
title('y_k(1) vs y_k(3)')
xlabel('y_k(1)')
ylabel('y_k(3)')

subplot(1,3,3)
scatter(GPS(:,2),GPS(:,3))
axis([-10 10 -20 20])
grid on
title('y_k(2) vs y_k(3)')
xlabel('y_k(2)')
ylabel('y_k(3)')

% b
R=cov(GPS)

%c
R=cov(GPS(1:3,:));
N = 3;                                                  % Number Of Times To Repeat
Ar = repmat(R, 1, N);                                   % Repeat Matrix
Ac = mat2cell(Ar, size(R,1), repmat(size(R,2),1,N));    % Create Cell Array Of Orignal Repeated Matrix
R = blkdiag(Ac{:}); % Desired Result
H=zeros(3*N,3);
for i=0:N-1
    H(3*i+1:3*i+3,:)=eye(3);
end
y=zeros(3*N,1);
for i=0:N-1
    y(3*i+1:3*i+3)=GPS(i+1,:);
end
x_3=inv(H'*inv(R)*H)*H'*inv(R)*y;
error_3=inv(H'*inv(R)*H);

R=cov(GPS(1:10,:));
N = 10;
Ar = repmat(R, 1, N);
Ac = mat2cell(Ar, size(R,1), repmat(size(R,2),1,N));
R = blkdiag(Ac{:});
H=zeros(3*N,3);
for i=0:N-1
    H(3*i+1:3*i+3,:)=eye(3);
end
y=zeros(3*N,1);
for i=0:N-1
    y(3*i+1:3*i+3)=GPS(i+1,:);
end
x_10=inv(H'*inv(R)*H)*H'*inv(R)*y;
error_10=inv(H'*inv(R)*H);

R=cov(GPS);
N = 100;
Ar = repmat(R, 1, N);
Ac = mat2cell(Ar, size(R,1), repmat(size(R,2),1,N));
R = blkdiag(Ac{:});
H=zeros(3*N,3);
for i=0:N-1
    H(3*i+1:3*i+3,:)=eye(3);
end
y=zeros(3*N,1);
for i=0:N-1
    y(3*i+1:3*i+3)=GPS(i+1,:);
end
x_100=inv(H'*inv(R)*H)*H'*inv(R)*y;
error_100=inv(H'*inv(R)*H);

% d
data=load('hw6problem5data.csv')';
N=length(data);
R=cov(data);
Ar = repmat(R, 1, N);
Ac = mat2cell(Ar, size(R,1), repmat(size(R,2),1,N));
R = blkdiag(Ac{:});
H=zeros(3*N,3);
for i=0:N-1
    H(3*i+1:3*i+3,:)=eye(3);
end
y=zeros(3*N,1);
for i=0:N-1
    y(3*i+1:3*i+3)=data(i+1,:);
end
x_real=inv(H'*inv(R)*H)*H'*inv(R)*y;
error_real=inv(H'*inv(R)*H);

% e
x_unw=mean(data)';
error_unw=inv(H'*H);

% f
% good ol' engineering knowhow
x=zeros(30,3);
P=zeros(3,3,30);
P(:,:,1)=2*eye(3);
x(1,:)=[5 -17 50];
H=eye(3);
for i=1:29
    R=cov(data(1:i+1,:));
    K=P(:,:,i)*H'*inv(H*P(:,:,i)*H'+R);
    x(i+1,:)=(x(i,:)'+K*(data(i+1,:)'-H*x(i,:)'))';
    P(:,:,i+1)=(eye(3)-K*H)*P(:,:,i)*(eye(3)-K*H)'+K*R*K';
end

%because MATLAB can't figure out the type of variable it wants to be
sigma1=zeros(30,1);
sigma2=zeros(30,1);
sigma3=zeros(30,1);

for i=1:30
    sigma1(i)=2*sqrt(P(1,1,i));
    sigma2(i)=2*sqrt(P(2,2,i));
    sigma3(i)=2*sqrt(P(3,3,i));
end

figure
subplot(3,1,1)
hold on
plot(linspace(1,30,30),x(:,1),'b')
plot(linspace(1,30,30),x(:,1)+sigma1(:),'b--')
plot(linspace(1,30,30),x(:,1)-sigma1(:),'b--')
xlabel('k')
ylabel('\zeta (m)')
legend('Estimated state','2\sigma')
title('RLLS Estimates for State')

subplot(3,1,2)
hold on
plot(linspace(1,30,30),x(:,2),'r')
plot(linspace(1,30,30),x(:,2)+sigma2(:),'r--')
plot(linspace(1,30,30),x(:,2)-sigma2(:),'r--')
xlabel('k')
ylabel('\eta (m)')
legend('Estimated state','2\sigma')

subplot(3,1,3)
hold on
plot(linspace(1,30,30),x(:,3),'g')
plot(linspace(1,30,30),x(:,3)+sigma3(:),'g--')
plot(linspace(1,30,30),x(:,3)-sigma3(:),'g--')
xlabel('k')
ylabel('z (m)')
legend('Estimated state','2\sigma')