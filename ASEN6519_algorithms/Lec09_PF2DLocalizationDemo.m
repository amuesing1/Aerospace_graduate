%Lec09_PF2DLocalizationDemo.m
%Use range-only measurements taken every nskip time steps 
%%(i.e. measurements NOT received every single time step)
%%Black dots = weighted Particle Filter samples 
%%Black ellipse/cross = weighted sample mean and covar from Particle Filter
%%Pink crosses = noisy range beacons
%%Blue path = true robot trajectory

clc,clear,
close all
rng(1928374655)

%% 0. Set up
dt = 0.02; %should be small when using Euler integration!
tvec = dt:dt:100;
whist = 0.2*sin(0.1*tvec);%0.45*ones(size(tvec)); %angular velocity profile, in rad/s
qhist = 0.5*ones(size(tvec)); %linear velocity profile, in m/s
x0true = [0.1,0.1,0]'; %initial state: x,y,theta

Q = diag([0.1; 0.1; 0.05]);%diag([0.1; 0.1; 0.05]); %process noise variances
R = 1.5; %true range measurement noise variances

% landmarks = [ ...
%     -1.2856    5.8519
%     1.5420    2.1169
%    -0.1104    1.7926
%     4.2603    9.7480
%     2.6365   12.9204
%    -3.5036    7.7518
%    -1.6228   10.2106
%    -9.8876    1.2568
%     2.1522    0.5491
%    -7.3594   11.9139];
landmarks = [...
    -1.2856-2.5    6.4964
    1.5420-2.5    8.3772
   -0.1104-2.5    0.1124
    4.2603-2.5   12.1522
    2.6365-2.5    2.6406];
nLandmarks = size(landmarks,1);

%% 1. Simulate forward truth model using Euler integration
xtruehist = zeros(3,length(tvec)+1);
xtruehist(:,1) = x0true;

zhist = nan(nLandmarks,length(tvec)+1);

for kk=1:length(tvec)
    v = mvnrnd(zeros(3,1),Q,1);
    xtruehist(1,kk+1) = xtruehist(1,kk) + dt*qhist(kk)*cos(xtruehist(3,kk)) + dt*v(1);
    xtruehist(2,kk+1) = xtruehist(2,kk) + dt*qhist(kk)*sin(xtruehist(3,kk)) + dt*v(2);
    xtruehist(3,kk+1) = xtruehist(3,kk) + dt*whist(kk) + dt*v(3);
    
    %%Simulate range/bearing measurements to origin landmark
    for ll=1:nLandmarks
        r = mvnrnd(zeros(1,1),R,1);
        zhist(ll,kk+1) =sqrt( (landmarks(ll,1)-xtruehist(1,kk+1)).^2 + ...
                              (landmarks(ll,2)-xtruehist(2,kk+1)).^2 ) + r;
        zhist(ll,kk+1) = max(zhist(ll,kk+1),0); %enforce non-negativity
    end
end
figure(),hold on
plot(xtruehist(1,:),xtruehist(2,:),'b')
plot(xtruehist(1,1),xtruehist(2,1),'bo','MarkerSize',15,'LineWidth',3)
plot(xtruehist(1,end),xtruehist(2,end),'bx','MarkerSize',15,'LineWidth',3)
plot(landmarks(:,1),landmarks(:,2),'m+','MarkerSize',15,'LineWidth',3)
%%Plot range measurements:
% for kk=1:length(tvec)
%     if mod(kk,20)==0
%     viscircles(repmat([xtruehist(1,kk),xtruehist(2,kk)],[nLandmarks 1]),...
%                zhist(:,kk),'EdgeColor','g','DrawBackgroundCircle',false,...
%                'LineWidth',0.5,'LineStyle','--')
%     end
% end

%% 2. Run PF with Euler integration
xhat = zeros(3,length(tvec)+1);
Phat = zeros(3,3,length(tvec)+1);
xbar = nan(3,length(tvec)+1);
Pbar = nan(3,3,length(tvec)+1);
eta = nan(2,length(tvec)+1);

Qpf = diag([0.15;0.15; 0.1]); %process noise tuning
Rpf = 0.5; %measurement noise tuning

nx = 3; %number of states
nz = 2; %number of sensor measurements per step
nv = length(Q); %number of process noise inputs
nw = length(R); %number of sensor noise inputs

nSamples = 300;
xsamplehist = zeros(3,nSamples,length(tvec)+1);
xsamplehist(:,:,1) = -.25+0.5*rand(3,nSamples); %start from large uniform distribution for initial condition
wsamplehist(1,:) = (1/nSamples)*ones(1,nSamples); %initialize particle weights

nskip = 150; %how often to do a measurement update: must be >=1
ess = nSamples;
for kk=1:length(tvec)
    
    %%Prediction step
    vpart = mvnrnd(zeros(1,3),Qpf,nSamples)';
    xsamplehist(1,:,kk+1) = xsamplehist(1,:,kk) + dt*qhist(kk)*cos(xsamplehist(3,:,kk)) + dt*vpart(1,:);
    xsamplehist(2,:,kk+1) = xsamplehist(2,:,kk) + dt*qhist(kk)*sin(xsamplehist(3,:,kk)) + dt*vpart(2,:);
    xsamplehist(3,:,kk+1) = xsamplehist(3,:,kk) + dt*whist(kk) + dt*vpart(3,:);
    %%Compute mean and variance from particles
    xyParticleMu = mean(squeeze(xsamplehist(1:2,:,kk+1)),2);
    xyParticleCov = cov(squeeze(xsamplehist(1:2,:,kk+1))');
    if kk==1
        hold on
        hpart = plot(xsamplehist(1,:,kk+1),xsamplehist(2,:,kk+1),'k.');
        [x2spart,y2spart] = calc_gsigma_ellipse_plotpoints(xyParticleMu,xyParticleCov,2,100);
        hellpart = plot(x2spart,y2spart,'k');
        hmupart = plot(xyParticleMu(1),xyParticleMu(2),'k+','MarkerSize',15,'LineWidth',2);
    elseif mod(kk,10)==0
        set(hpart,'XData',xsamplehist(1,:,kk+1),'YData',xsamplehist(2,:,kk+1))
        [x2spart,y2spart] = calc_gsigma_ellipse_plotpoints(xyParticleMu,xyParticleCov,2,100);
        set(hellpart,'XData',x2spart,'YData',y2spart)
        set(hmupart,'XData',xyParticleMu(1),'YData',xyParticleMu(2))
        title(['kk = ',num2str(kk+1),', ESS @ last meas update =',num2str(ess)])
    end
    
    %%Measurement update step
    if mod(kk+1,nskip)==0
        for ll=1:nLandmarks
            currzll = zhist(ll,kk+1);
            rhobarll = ...
                sqrt(sum((repmat(landmarks(ll,:),[nSamples 1])- xsamplehist(1:2,:,kk+1)').^2,2));
            lkll = mvnpdf(currzll,rhobarll,Rpf);
            
            wsamplehist = wsamplehist.*lkll'; %redo in log form to avoid underflow issues
            wsamplehist = wsamplehist./sum(wsamplehist);
            
        end
        %%Resample particles if necessary
            coeffvar = var(wsamplehist)/(mean(wsamplehist))^2;
            %%compute effective sample size for IS
            ess = nSamples/(1+coeffvar);
        title(['kk = ',num2str(kk+1),', ESS =',num2str(ess)])
        if ess<0.5*nSamples
            [sampsOut,wtsOut] = resampleParticles(xsamplehist(:,:,kk+1),wsamplehist);
            xsamplehist(:,:,kk+1) = sampsOut;
            wsamplehist = wtsOut;
        end
    end
    axis tight %axis([-10 10 -19 19])    
    pause(0.0025)
end