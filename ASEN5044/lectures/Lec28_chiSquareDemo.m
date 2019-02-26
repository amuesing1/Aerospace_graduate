%chiSquareDemo.m
%ASEN 6519 Model-based Parameter and State Estimation, Spring 2015
% Nisar Ahmed
%Demonstrate chi-square distribution for norms of Gaussian RVs.

clc,clear,
close all
rng(105) %seed random # generator

%%Draw dependent Gaussian samples
P = [3 2;
     2 6];

invP = 0.5*(inv(P) + inv(P)') ;
x = mvnrnd(zeros(2,1),P,5000);

figure(), hold on
plot(x(:,1),x(:,2),'bx','MarkerSize',8,'LineWidth',2)
xlabel('e_1','FontSize',14)
ylabel('e_2','FontSize',14)
title('Correlated Gaussian Samples','FontSize',18)
%%Compute sample covariance
covX = cov(x)

%%Apply whitening transform
S = chol(invP);
g = (S*x')';

figure(), hold on
plot(g(:,1),g(:,2),'rx','MarkerSize',8,'LineWidth',2)
xlabel('g_1','FontSize',14)
ylabel('g_2','FontSize',14)
title('Decorrelated ''Whitened'' Gaussian Samples','FontSize',18)
%%Compare sample covariance
covG = cov(g)

%% Get histogram of norms
figure(),
hist(sum(g.^2,2),100);
title('Histogram for g^2 samples','FontSize',14)
set(gca,'FontSize',14)

%%Plot predicted chi square distribution
nf = size(g,2);
v2vals = 0:0.01:30;
figure()
pdfv2 = chi2pdf(v2vals,nf);
plot(v2vals,pdfv2,'m','LineWidth',3)
title('Chi-square pdf with 2 degrees of freedom','FontSize',14)
set(gca,'FontSize',14)
xlabel('g^2')
ylabel('p(g^2)')

%% Show General chi-square pdfs
figure(), hold on
dofs = 2:2:18;
ltext = {};
for dd=dofs
    pdfc2 = chi2pdf(v2vals,dd);
    plot(v2vals,pdfc2,'Color',[dd/max(dofs) (max(dofs)-dd)/max(dofs) 1],'LineWidth',3)
    ltext{end+1} = ['chi2pdf, DOFs = ',num2str(dd)];
end
legend(ltext)
title('Chi-square pdf for varying DOFS','FontSize',14)
set(gca,'FontSize',14)
xlabel('g^2')
ylabel('p(g^2)')