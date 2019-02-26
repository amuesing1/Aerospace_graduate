%lec14_conditionalpdfBayesRuledemo.m

clc,clear,close all

rng(300)

%% Case 1: uniform slanted slab
% xrange = 0:0.01:13;
% yrange = 0:0.01:17;
% 
% [X,Y]=meshgrid(xrange,yrange);
% 
% XY = [X(:),Y(:)];
% 
% pX = pdf('unif',X(:),1,11);
% pY_X = pdf('unif',Y(:),X(:),X(:)+5);
% 
% pXYjoint = pX.*pY_X;
% figure(),
% surf(X,Y,reshape(pXYjoint,size(X)),'EdgeColor','none')

%% Case 2: more complex pdf with y bounds depending quadratically on x
dx=0.05;
xrange = 0:dx:14;
yrange = 0:dx:13^2;

[X,Y]=meshgrid(xrange,yrange);

XY = [X(:),Y(:)];

pX = pdf('unif',X(:),1,11);
pY_X = pdf('unif',Y(:),X(:),(X(:)+1).^2);

pXYjoint = pX.*pY_X;
figure(),
surf(X,Y,reshape(pXYjoint,size(X)),'EdgeColor','none')
xlim([0 12])
ylim([0 12^2])
xlabel('x')
ylabel('y')
title('Joint pdf p(x,y) = U_x[1,11] U_{y|x}[x,(x+1)^2]')

%%show prior for x only
pxPrior = pdf('unif',xrange,1,11);
figure(),
plot(xrange,pxPrior,'r','LineWidth',3)
axis([0 14 0 0.2])
title('Prior p(x) = U[1,11]')
xlabel('x')
ylabel('p(x), prior')


%%find likelihood for given observation value of y
yobs = 114
py4_x = pdf('unif',yobs,xrange,(xrange+1).^2);
figure(),
plot(xrange,py4_x,'b','LineWidth',3)
xlim([0 14])
title(['Likelihood p(y=',num2str(yobs),'|x) = U[x,(x+1)^2]'])
xlabel('x')
ylabel(['p(y=',num2str(yobs),'|x), observation likelihood'])

%%show posterior for x
pxPost_num = pxPrior.*py4_x;
pxPost_den = sum(pxPost_num)*dx;
pxPost = pxPost_num./pxPost_den; 
figure(),
plot(xrange,pxPost,'m','LineWidth',3)
xlabel('x')
ylabel(['p(x|y=',num2str(yobs),'), posterior'])
title(['Bayes'' Posterior p(x|y=',num2str(yobs),')'])

