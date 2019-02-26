close all; clear all; clc;
%% part a and b
% a
omega=0.045;
dt=0.5;
% sanity check that I got A right
A=[0 1 0 0; 0 0 0 -omega; 0 0 0 1; 0 omega 0 0];
derived_expm=expm(A*dt);
trueexpm=[1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega;
    0 cos(omega*dt) 0 -sin(omega*dt);
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega;
    0 sin(omega*dt) 0 cos(omega*dt)];
% I did

% b
F=trueexpm;
m0=[0;85*cos(pi/4);0;-85*sin(pi/4)];
P0=eye(4).*[10;2;10;2];
m=zeros(4,300);
P=zeros(4,4,300);

for k=1:300
    m(:,k)=F^k*m0;
    P(:,:,k)=F^k*P0*(F')^k;
end
%because MATLAB can't figure out the type of variable it wants to be
sigma1=zeros(300,1);
sigma2=zeros(300,1);
sigma3=zeros(300,1);
sigma4=zeros(300,1);

for i=1:300
    sigma1(i)=2*sqrt(P(1,1,i));
    sigma2(i)=2*sqrt(P(2,2,i));
    sigma3(i)=2*sqrt(P(3,3,i));
    sigma4(i)=2*sqrt(P(4,4,i));
end

figure
subplot(4,1,1)
hold on
plot(linspace(1,150,300),m(1,:),'b')
plot(linspace(1,150,300),m(1,:)'+sigma1,'b--')
plot(linspace(1,150,300),m(1,:)'-sigma1,'b--')
xlabel('Time (s)')
ylabel('East pos (m)')
title('State elements over time')

subplot(4,1,2)
hold on
plot(linspace(1,150,300),m(2,:),'r')
plot(linspace(1,150,300),m(2,:)'+sigma2,'r--')
plot(linspace(1,150,300),m(2,:)'-sigma2,'r--')
xlabel('Time (s)')
ylabel('East vel (m/s)')

subplot(4,1,3)
hold on
plot(linspace(1,150,300),m(3,:),'g')
plot(linspace(1,150,300),m(3,:)'+sigma3,'g--')
plot(linspace(1,150,300),m(3,:)'-sigma3,'g--')
xlabel('Time (s)')
ylabel('North pos (m)')

subplot(4,1,4)
hold on
plot(linspace(1,150,300),m(4,:),'c')
plot(linspace(1,150,300),m(4,:)'+sigma4,'c--')
plot(linspace(1,150,300),m(4,:)'-sigma4,'c--')
xlabel('Time (s)')
ylabel('North vel (m/s)')

figure
subplot(4,1,1)
plot(linspace(1,150,300),sigma1)
xlabel('Time (s)')
ylabel('East pos (m^2)')
title('2\sigma bounds on state elements')

subplot(4,1,2)
plot(linspace(1,150,300),sigma2)
xlabel('Time (s)')
ylabel('East vel (m/s)^2')

subplot(4,1,3)
plot(linspace(1,150,300),sigma3)
xlabel('Time (s)')
ylabel('North pos (m^2)')

subplot(4,1,4)
plot(linspace(1,150,300),sigma4)
xlabel('Time (s)')
ylabel('North vel (m/s)^2')

figure
hold on
plot(m(1,:),m(3,:))
S=chol(P0,'lower');
sam_size=500;
x_store=zeros(2,sam_size*10);
for j=1:sam_size
    x0=m0+S*randn(4,1);
    for k=1:300
        if rem(k,30)==0
            x=F^k*x0;
            x_store(:,10*(j-1)+k/30)=[x(1),x(3)];
        end
    end
end
scatter(x_store(1,:),x_store(2,:),'r.')
for i=1:300
    if rem(i,30)==0
        covar(1,1)=P(1,1,i);
        covar(2,2)=P(3,3,i);
        covar(1,2)=P(1,3,i);
        covar(2,1)=P(1,3,i);
        r_ellipse=error_ellipse(covar);
        plot(r_ellipse(:,1) + m(1,i),r_ellipse(:,2) + m(3,i),'black-')
    end
end
grid on
axis equal
xlabel('East Pos (m)')
ylabel('North Pos (m)')
title('Error ellipses and MC samples')

%% part c

ma=[0;85*cos(pi/4);0;-85*sin(pi/4)];
Pa=eye(4).*[10;4;10;4];
mb=[3200;85*cos(pi/4);3200;-85*sin(pi/4)];
Pb=eye(4).*[11;3.5;11;3.5];


omega=0.045;
dt=0.5;
Fa=[1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega;
    0 cos(omega*dt) 0 -sin(omega*dt);
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega;
    0 sin(omega*dt) 0 cos(omega*dt)];

omega=-0.045;
Fb=[1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega;
    0 cos(omega*dt) 0 -sin(omega*dt);
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega;
    0 sin(omega*dt) 0 cos(omega*dt)];

ma_all=zeros(4,300);
Pa_all=zeros(4,4,300);
mb_all=zeros(4,300);
Pb_all=zeros(4,4,300);
probs=zeros(300,1);
for k=1:300
    mfull=Fa^k*ma-Fb^k*mb;
    mc=[mfull(1);mfull(3)];
    P=Fa^k*Pa*(Fa')^k+Fb^k*Pb*(Fb')^k;
    Pc=[P(1,1) P(1,3);P(3,1) P(3,3)];
    probs(k)=mvncdf([100,100],mc',Pc)-mvncdf([-100,-100],mc',Pc);
    
    ma_all(:,k)=Fa^k*ma;
    Pa_all(:,:,k)=Fa^k*Pa*(Fa')^k;
    mb_all(:,k)=Fb^k*mb;
    Pb_all(:,:,k)=Fb^k*Pb*(Fb')^k;
end

%because MATLAB can't figure out the type of variable it wants to be
sigmaa1=zeros(300,1);
sigmaa3=zeros(300,1);
sigmab1=zeros(300,1);
sigmab3=zeros(300,1);

for i=1:300
    sigmaa1(i)=2*sqrt(Pa_all(1,1,i));
    sigmaa3(i)=2*sqrt(Pa_all(3,3,i));
    sigmab1(i)=2*sqrt(Pb_all(1,1,i));
    sigmab3(i)=2*sqrt(Pb_all(3,3,i));
end


figure
subplot(2,1,1)
hold on
x=linspace(1,150,300);
yyaxis left
h1=plot(x,ma_all(1,:),'b');
h2=plot(x,ma_all(1,:)'+sigmaa1,'b--');
plot(x,ma_all(1,:)'-sigmaa1,'b--')
h3=plot(x,mb_all(1,:),'g');
h4=plot(x,mb_all(1,:)'+sigmab1,'g--');
plot(x,mb_all(1,:)'-sigmab1,'g--')
ylabel('East pos (m)')
xlabel('Time (s)')

yyaxis right
h5=plot(x,probs);
ylabel('p(r_c)')
legend([h1,h2,h3,h4,h5],{'Aircraft A','2\sigma bounds','Aircraft B','2\sigma bounds','p(r_c(k))'})
title('East Pos of A,B and Prob of Collision')

subplot(2,1,2)
hold on
x=linspace(1,150,300);
yyaxis left
h1=plot(x,ma_all(3,:),'b');
h2=plot(x,ma_all(3,:)'+sigmaa3,'b--');
plot(x,ma_all(3,:)'-sigmaa3,'b--')
h3=plot(x,mb_all(3,:),'g');
h4=plot(x,mb_all(3,:)'+sigmab3,'g--');
plot(x,mb_all(3,:)'-sigmab3,'g--')
ylabel('North pos (m)')
xlabel('Time (s)')

yyaxis right
h5=plot(x,probs);
ylabel('p(r_c(k))')
legend([h1,h2,h3,h4,h5],{'Aircraft A','2\sigma bounds','Aircraft B','2\sigma bounds','p(r_c(k))'})
title('North Pos of A,B and Prob of Collision')