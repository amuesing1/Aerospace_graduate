% Jeremy Muesing
% ASEN 5044
% HW3

close all; clear all; clc;

%% Problem 1
% a
Ahat=[0 1 0 0 0 0;-2 0 1 0 -1 0; 0 0 0 1 0 0;...
    1 0 -2 0 1 1; 0 0 0 0 0 0; 0 0 0 0 0 0];
dt=.05;

FGHM=expm(Ahat*dt);

A=[0 1 0 0;-2 0 1 0; 0 0 0 1; 1 0 -2 0];
e=eig(A);

(pi/dt)-2*max(abs(e));

% c
F=FGHM(1:4,1:4);
G=FGHM(1:4,5:6);
H=[1 0 0 0; 0 1 0 -1];
O=[H*F; H*F^2; H*F^3; H*F^4; H*F^5];
zslot=zeros(2);
P=[H*G zslot zslot; H*F*G H*G zslot; H*(F^2)*G H*F*G H*G];
load('hw3problem1data.mat');
Y=zeros(6,1);
u=zeros(6,1);
for i=1:3
    Y(2*i-1)=Ydata(i,1);
    Y(2*i)=Ydata(i,2);
    u(2*i-1)=Udata(i,1);
    u(2*i)=Udata(i,2);
end

x0=inv(O'*O)*O'*(Y-P*u);
x=zeros(100,4);
y=zeros(100,2);
x(1,:)=x0;
count=1;
for i=0:.05:5
    x(count+1,:)=F*x(count,:)'+G*[sin(i); 0.1*cos(i)];
    y(count,:)=H*x(count,:)';
    count=count+1;
end

figure(1)
subplot(3,1,1)
plot(linspace(.05,5,5/.05),x(2:end-1,:))
legend('Pos m_1','Vel m_1','Pos m_2','Vel m_2')
title('X states')

subplot(3,1,2)
plot(linspace(.05,5,5/.05),y(2:end,:))
legend('Pos m_1','Pos m_2')
title('Y states')

subplot(3,1,3)
plot(linspace(.05,5,5/.05),y(2:end,:)-Ydata)
legend('Pos m_1''-Pos m_1','Pos m_2''-Pos m_2')
title('Y(pred)-Y(real)')
xlabel('Time (sec)')

%% Problem 2
clear all;
% b, c & d

alpha=[0.75,0.75,1.25];
Beta=[1,1.5,1];
eigenvalues=zeros(3,2);

for i=1:length(alpha)
    F=[alpha(i) alpha(i); Beta(i)*(alpha(i)-1) Beta(i)*alpha(i)];
    G=[alpha(i); Beta(i)*alpha(i)];
    H=[1 1];
    eigenvalues(i,:)=eig(F);
    
    x0=[0 0;5 1];
    uk=[1 ;0];
    for j=1:length(x0)
        xk=zeros(31,2);
        yk=zeros(30,1);
        xk(1,:)=x0(j,:);
        for k=1:30
            xk(k+1,:)=F*xk(k,:)'+G*uk(j);
            if j==2
                yk(k)=H*xk(k,:)';
            end
        end
        if j==2
            for k=2:20
                O(k,:)=H*F^(k-1);
            end
            Y=yk(1:20);
            x0_est=inv(O'*O)*O'*Y;
            xk_pred=zeros(31,2);
            yk_pred=zeros(30,1);
            xk_pred(1,:)=x0_est';
            for k=1:30
                xk_pred(k+1,:)=F*xk_pred(k,:)'+G*uk(j);
                yk_pred(k)=H*xk_pred(k,:)';
            end
            figure(4)
            ax4=subplot(3,2,i*2-1);
            hold(ax4,'on')
            plot(linspace(0,30,30),yk)
            plot(linspace(0,30,30),yk_pred)
            if i==3
                xlabel('k (years)')
            end
            legend('y_k real','y_k pred')
            title(['\alpha =' num2str(alpha(i)) ', \beta =' num2str(Beta(i))])
            hold(ax4,'off')
            ax5=subplot(3,2,i*2);
            plot(linspace(0,30,30),yk-yk_pred)
            title('y_k Real-y_k Pred')
            if i==3
                xlabel('k (years)')
            end
        end
        figure(j+1)
        ax=subplot(3,1,i);
        hold(ax,'on')
        plot(linspace(0,30,31),xk(:,1))
        plot(linspace(0,30,31),xk(:,2))
        hold(ax,'off')
        legend('c_k','i_k')
        title(['\alpha =' num2str(alpha(i)) ', \beta =' num2str(Beta(i))])
        if i==3
            xlabel('k (years)')
        end
    end
end
eigenvalues

%% Problem 3
clear all;
%c
z=[100 20; 43.6658 39.2815; 40.5785 40.3382; 40.4093 40.3961; ...
    40.4 40.3993; 40.3995 40.3995];

% Generate observability matrix
y=zeros(5,2);
for i=1:length(z)-1
    y(i,1)=z(i+1,1)-z(i,1);
    y(i,2)=z(i+1,2)-z(i,2);
end
Y=zeros(10,1);
for i=1:5
    Y(2*i-1)=y(i,1);
    Y(2*i)=y(i,2);
end
F=eye(2);
for i=1:5
    H(:,:,i)=[z(i,1)-z(i,2) 0; 0 z(i,1)-z(i,2)];
end
O=[H(:,:,1);H(:,:,2)*F;H(:,:,3)*F^2;H(:,:,4)*F^3;H(:,:,5)*F^4];
x0=inv(O'*O)*O'*Y