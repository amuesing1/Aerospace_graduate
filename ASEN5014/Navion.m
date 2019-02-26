% Navion lateral dynamics
% JSH 10/23/18

clear all global;

% Lateral Dynamics, u0 = 132 kts, h = 5000 ft
A = [-0.1473 -0.0014 -0.9918 0.1470; ...
     -28.749 -12.41 2.535 0; ...
     10.119 -0.382 -1.259 0; ...
     0 1 0 0];
B = [0.0708 2.548 -4.597 0]';
C = eye(4);
D = [0 0 0 0]';
G_lat = ss(A,B,C,D);

% Compute eigenstructure
[V,lambda] = eig(A);
V1 = V(:,1);    % Roll
V2 = V(:,2);    % Dutch Roll 1
V3 = V(:,3);    % Dutch Roll 2
V4 = V(:,4);    % Spiral    

figure(1); 
compass(V1);

damp(G_lat);    % or damp(A)

% Impulse Response
figure(2); clf;
impulse(G_lat,8); grid on;
title('Impulse Response - Lateral Dynamics');

% Initial condition response
figure(3); clf;
initial(G_lat,real(V2),8); grid on; hold on;
initial(G_lat,imag(V2),8); 
title('Initial Condition Response - Dutch Roll');
legend('Real','Imag');
figure(4); clf;
initial(G_lat,real(V1),0.5); grid on;
title('Initial Condition Response - Roll');
figure(5); clf;
initial(G_lat,real(V4),300); grid on;
title('Initial Condition Response - Spiral');
figure(6); clf;
initial(G_lat,B,8); grid on;
title('Initial Condition Response - B Matrix');

% Forced response
x0 = [0 0 0 0]';
[U,T] = gensig('sin',2,6);
[Y,T1,X] = lsim(G_lat,U,T,x0);
figure(7); clf;
subplot(4,1,1); plot(T,Y(:,1),'m');
title('Forced Response');
hold on; plot(T,U,'k--'); grid on; ylabel('\Delta\beta');
subplot(4,1,2); plot(T,Y(:,2),'b');
hold on; plot(T,U,'k--'); grid on; ylabel('\Deltap');
subplot(4,1,3); plot(T,Y(:,3),'r');
hold on; plot(T,U,'k--'); grid on; ylabel('\Deltar');
subplot(4,1,4); plot(T,Y(:,4),'g');
hold on; plot(T,U,'k--'); grid on; ylabel('\Delta\phi');
xlabel('Time(sec)');

t1 = 32; t2 = 32;
U2 = [zeros(1,t1) ones(1,t2) zeros(1,193-t1-t2)]';
[Y,T1,X] = lsim(G_lat,U2,T,x0);
figure(8); clf;
subplot(4,1,1); plot(T,Y(:,1),'m');
title('Forced Response');
hold on; plot(T,U2,'k--'); grid on; ylabel('\Delta\beta');
subplot(4,1,2); plot(T,Y(:,2),'b');
hold on; plot(T,U2,'k--'); grid on; ylabel('\Deltap');
subplot(4,1,3); plot(T,Y(:,3),'r');
hold on; plot(T,U2,'k--'); grid on; ylabel('\Deltar');
subplot(4,1,4); plot(T,Y(:,4),'g');
hold on; plot(T,U2,'k--'); grid on; ylabel('\Delta\phi');
xlabel('Time(sec)');

% Frequency response
s = tf('s');
G = C*inv(s*eye(4)-A)*B;
figure(9); clf;
bode(G(1),'m',G(2),'b',G(3),'r',G(4),'g',{1e-1,1e2}); 
grid on;
legend('to \Delta\beta', 'to \Deltap','to \Deltar','to \Delta\phi');

figure(10); clf
sigma(G_lat,'k--'); hold on; grid on;
[mag, phase, omega] = bode(G_lat);
mag = squeeze(mag); phase = squeeze(phase);
omega_min = 1; omega_max = 1e3;
plot(omega, 20*log10(mag(1,:)),'m');
plot(omega, 20*log10(mag(2,:)),'b');
plot(omega, 20*log10(mag(3,:)),'r');
plot(omega, 20*log10(mag(4,:)),'g');
set(gca, 'XScale','log')
xlabel('Frequency (rad/sec)')
ylabel('Magnitude (dB)')
axis([1e-1 1e2 -80 20]);
legend('\sigma(G)','to \Delta\beta', 'to \Deltap','to \Deltar','to \Delta\phi');