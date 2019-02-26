close all; clear all; clc;
% 3b
omega=0.045;
dt=0.5;
F=[1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega;
    0 cos(omega*dt) 0 -sin(omega*dt);
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega;
    0 sin(omega*dt) 0 cos(omega*dt)];
data=load('midterm2_problem3b.mat')';
data=data.yaHist;
N=length(data);
R=cell(1,N);
for k=1:N
    R{k}=[75 7.5;7.5 75]+[12.5*sin(k/10) 25.5*sin(k/10);25.5*sin(k/10) 12.5*cos(k/10)];
end
R = blkdiag(R{:});
H=zeros(2*N,4);
for i=0:N-1
    H(2*i+1:2*i+2,:)=[1 0 0 0;0 0 1 0]*F^(i+1);
end
y=zeros(2*N,1);
for i=0:N-1
    y(2*i+1:2*i+2)=data(:,i+1);
end
x_0=inv(H'*inv(R)*H)*H'*inv(R)*y;
error_0=inv(H'*inv(R)*H);

% c
data=load('midterm2_problem3c.mat')';
data=data.yaugHist;
N=length(data);
x=zeros(N,8);
P=zeros(8,8,N);
P(:,:,1)=200*eye(8);
x(1,:)=[-200 -80 0 -160 3000 50 3000 -250];
H=[1 0 0 0 0 0 0 0;0 0 1 0 0 0 0 0;1 0 0 0 -1 0 0 0;0 0 1 0 0 0 -1 0];
Fa=[1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega;
    0 cos(omega*dt) 0 -sin(omega*dt);
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega;
    0 sin(omega*dt) 0 cos(omega*dt)];
omega=-0.045;
Fb=[1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega;
    0 cos(omega*dt) 0 -sin(omega*dt);
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega;
    0 sin(omega*dt) 0 cos(omega*dt)];
F=[Fa zeros(4);zeros(4) Fb];
for i=1:N-1
    Ra=[75 7.5;7.5 75]+[12.5*sin(k/10) 25.5*sin(k/10);25.5*sin(k/10) 12.5*cos(k/10)];
    R=[Ra zeros(2);zeros(2) [8000 500;500 8000]];
    K=P(:,:,i)*(H*F^i)'*inv((H*F^i)*P(:,:,i)*(H*F^i)'+R);
    x(i+1,:)=(x(i,:)'+K*(data(:,i)-(H*F^i)*x(i,:)'))';
    P(:,:,i+1)=(eye(8)-K*(H*F^i))*P(:,:,i)*(eye(8)-K*(H*F^i))'+K*R*K';
end

%because MATLAB can't figure out the type of variable it wants to be
sigma1=zeros(N,1);
sigma2=zeros(N,1);
sigma3=zeros(N,1);
sigma4=zeros(N,1);
sigma5=zeros(N,1);
sigma6=zeros(N,1);
sigma7=zeros(N,1);
sigma8=zeros(N,1);


for i=1:N
    sigma1(i)=2*sqrt(P(1,1,i));
    sigma2(i)=2*sqrt(P(2,2,i));
    sigma3(i)=2*sqrt(P(3,3,i));
    sigma4(i)=2*sqrt(P(4,4,i));
    sigma5(i)=2*sqrt(P(5,5,i));
    sigma6(i)=2*sqrt(P(6,6,i));
    sigma7(i)=2*sqrt(P(7,7,i));
    sigma8(i)=2*sqrt(P(8,8,i));
end

figure
subplot(2,2,1)
hold on
plot(linspace(1,N,N),x(:,1),'b')
plot(linspace(1,N,N),x(:,1)+sigma1(:),'b--')
plot(linspace(1,N,N),x(:,1)-sigma1(:),'b--')
xlabel('k')
ylabel('\zeta_A (m)')
legend('Estimated state','2\sigma')
title('\zeta_A(0)')

subplot(2,2,2)
hold on
plot(linspace(1,N,N),x(:,2),'b')
plot(linspace(1,N,N),x(:,2)+sigma2(:),'b--')
plot(linspace(1,N,N),x(:,2)-sigma2(:),'b--')
xlabel('k')
ylabel('\zeta_A vel (m/s)')
legend('Estimated state','2\sigma')
title('\zeta_A vel(0)')

subplot(2,2,3)
hold on
plot(linspace(1,N,N),x(:,3),'b')
plot(linspace(1,N,N),x(:,3)+sigma3(:),'b--')
plot(linspace(1,N,N),x(:,3)-sigma3(:),'b--')
xlabel('k')
ylabel('\eta_A (m)')
legend('Estimated state','2\sigma')
title('\eta_A(0)')

subplot(2,2,4)
hold on
plot(linspace(1,N,N),x(:,4),'b')
plot(linspace(1,N,N),x(:,4)+sigma4(:),'b--')
plot(linspace(1,N,N),x(:,4)-sigma4(:),'b--')
xlabel('k')
ylabel('\eta_A vel (m/s)')
legend('Estimated state','2\sigma')
title('\eta_A vel(0)')

figure
subplot(2,2,1)
hold on
plot(linspace(1,N,N),x(:,5),'b')
plot(linspace(1,N,N),x(:,5)+sigma5(:),'b--')
plot(linspace(1,N,N),x(:,5)-sigma5(:),'b--')
xlabel('k')
ylabel('\zeta_B (m)')
legend('Estimated state','2\sigma')
title('\zeta_B(0)')

subplot(2,2,2)
hold on
plot(linspace(1,N,N),x(:,6),'b')
plot(linspace(1,N,N),x(:,6)+sigma6(:),'b--')
plot(linspace(1,N,N),x(:,6)-sigma6(:),'b--')
xlabel('k')
ylabel('\zeta_B vel (m/s)')
legend('Estimated state','2\sigma')
title('\zeta_B vel(0)')

subplot(2,2,3)
hold on
plot(linspace(1,N,N),x(:,7),'b')
plot(linspace(1,N,N),x(:,7)+sigma7(:),'b--')
plot(linspace(1,N,N),x(:,7)-sigma7(:),'b--')
xlabel('k')
ylabel('\eta_B (m)')
legend('Estimated state','2\sigma')
title('\eta_B(0)')

subplot(2,2,4)
hold on
plot(linspace(1,N,N),x(:,8),'b')
plot(linspace(1,N,N),x(:,8)+sigma8(:),'b--')
plot(linspace(1,N,N),x(:,8)-sigma8(:),'b--')
xlabel('k')
ylabel('\eta_B vel (m/s)')
legend('Estimated state','2\sigma')
title('\eta_B vel(0)')

figure
subplot(2,2,1)
plot(linspace(1,N,N),sigma1)
xlabel('k')
ylabel('\zeta_A (m)')
title('2\sigma of \zeta_A')

subplot(2,2,2)
plot(linspace(1,N,N),sigma2)
xlabel('k')
ylabel('\zeta_A vel (m/s)')
title('2\sigma of \zeta_A vel')

subplot(2,2,3)
plot(linspace(1,N,N),sigma3)
xlabel('k')
ylabel('\eta_A pos (m)')
title('2\sigma of \eta_A')

subplot(2,2,4)
plot(linspace(1,N,N),sigma4)
xlabel('k')
ylabel('\eta_A vel (m/s)')
title('2\sigma of \eta_A vel')

figure
subplot(2,2,1)
plot(linspace(1,N,N),sigma5)
xlabel('k')
ylabel('\zeta_B (m)')
title('2\sigma of \zeta_B')

subplot(2,2,2)
plot(linspace(1,N,N),sigma6)
xlabel('k')
ylabel('\zeta_B vel (m/s)')
title('2\sigma of \zeta_B vel')

subplot(2,2,3)
plot(linspace(1,N,N),sigma7)
xlabel('k')
ylabel('\eta_B pos (m)')
title('2\sigma of \eta_B')

subplot(2,2,4)
plot(linspace(1,N,N),sigma8)
xlabel('k')
ylabel('\eta_B vel (m/s)')
title('2\sigma of \eta_B vel')

% 4b
f=0.8;
q=10;
sigma2=50;

Pk=zeros(1,50);
Pk(1)=sigma2;
for i=1:49
    Pk(i+1)=f*Pk(i)*f+q;
end

P=ones(1,i+1)*(q/(1-f^2));

figure
subplot(2,1,1)
hold on
plot(linspace(0,50,50),Pk)
plot(linspace(0,50,50),P)
legend('Covariance Prediction Update','Analytical Solution')
xlabel('k')
ylabel('P_\infty')
title('\sigma^2_0=50')

sigma2=10;

Pk=zeros(1,50);
Pk(1)=sigma2;
for i=1:49
    Pk(i+1)=f*Pk(i)*f+q;
end

subplot(2,1,2)
hold on
plot(linspace(0,50,50),Pk)
plot(linspace(0,50,50),P)
legend('Covariance Prediction Update','Analytical Solution')
xlabel('k')
title('\sigma^2_0=10')
ylabel('P_\infty')

% 4d
F=[0.99 0.2;0 -.76];
Q=[1 0.37; 0.37 2.5];
P=reshape(inv(eye(4)-kron(F,F))*reshape(Q,[4,1]),[2,2])
P_real=dlyap(F,Q)

% 4e
steps=400;
Pk=zeros(2,2,steps);
Pk(:,:,1)=10*eye(2);
for i=1:steps-1
    Pk(:,:,i+1)=F*Pk(:,:,i)*F'+Q;
end
P_real=ones(2,2,steps).*P_real;
% MATLAB sucks
sigma1=reshape(Pk(1,1,:),[1,steps]);
sigma2=reshape(Pk(1,2,:),[1,steps]);
sigma3=reshape(Pk(2,1,:),[1,steps]);
sigma4=reshape(Pk(2,2,:),[1,steps]);

sigma1_r=reshape(P_real(1,1,:),[1,steps]);
sigma2_r=reshape(P_real(1,2,:),[1,steps]);
sigma3_r=reshape(P_real(2,1,:),[1,steps]);
sigma4_r=reshape(P_real(2,2,:),[1,steps]);

figure
subplot(2,2,1)
hold on
plot(linspace(0,steps,steps),sigma1)
plot(linspace(0,steps,steps),sigma1_r)
legend('Covariance Prediction Update','Analytical Solution')
xlabel('k')
ylabel('P_{1,1}')
title('P_{1,1}')

subplot(2,2,2)
hold on
plot(linspace(0,steps,steps),sigma2)
plot(linspace(0,steps,steps),sigma2_r)
legend('Covariance Prediction Update','Analytical Solution')
xlabel('k')
ylabel('P_{1,2}')
title('P_{1,2}')

subplot(2,2,3)
hold on
plot(linspace(0,steps,steps),sigma3)
plot(linspace(0,steps,steps),sigma3_r)
legend('Covariance Prediction Update','Analytical Solution')
xlabel('k')
ylabel('P_{2,1}')
title('P_{2,1}')

subplot(2,2,4)
hold on
plot(linspace(0,steps,steps),sigma4)
plot(linspace(0,steps,steps),sigma4_r)
legend('Covariance Prediction Update','Analytical Solution')
xlabel('k')
ylabel('P_{2,2}')
title('P_{2,2}')

%% AQ1
%b
figure
N=100;
steps=[10,50,180,300,600];
p0=zeros(N,1);
p0(3)=1/3;
p0(19)=1/3;
p0(35)=1/3;
count=1;
for k=1:steps(end)
    p_min=p0;
    for i=1:N
        post=eps;
        for j=1:N
            post=post+p0(j)*likelihoods(i,j);
        end
        p_min(i)=post;
    end
    p0=p_min/sum(p_min);
    if ismember(k,steps)
        subplot(2,3,count)
        bar(p0)
        count=count+1;
        title(strcat('Steps=',num2str(k)))
        ylabel('p(\zeta)')
        xlabel('\zeta')
    end
end

% c
load('midterm2_problemAQ1.mat')
figure
N=100;
r=25;
steps=[7,50,180,300,350];
p0=zeros(N,1);
p0(3)=1/3;
p0(19)=1/3;
p0(35)=1/3;
count=1;
MMSE=zeros(1,5);
MAP=zeros(1,5);
for k=1:steps(end)
    p_min=p0;
    for i=1:N
        post=eps;
        for j=1:N
            post=post+p0(j)*likelihoods(i,j);
        end
        p_min(i)=post;
    end
    p_min=p_min/norm(p_min);
    for j=1:N
        a=max(1,j-r);
        b=min(N,j+r);
        like=pdf('unif',zMeasHist(k+1),a,b);
        p_min(j)=p_min(j)*like;
    end
    p_min=p_min/sum(p_min);
    p0=p_min;
    if ismember(k,steps)
        subplot(2,3,count)
        bar(p_min)
        [~,argmax]=max(p_min);
        MMSE(count)=sum(linspace(1,N,N)*p_min);
        MAP(count)=argmax;
        count=count+1;
        title(strcat('Steps=',num2str(k)))
        ylabel('p(\zeta)')
        xlabel('\zeta')
    end
end
true=zetaHist(steps);
figure
hold on
scatter(steps,MMSE,50,'filled','s')
scatter(steps,MAP,50,'filled','d')
scatter(steps,true,50,'filled')
legend('MMSE','MAP','ground truth')
xlabel('k')
ylabel('\zeta(k)')
grid on