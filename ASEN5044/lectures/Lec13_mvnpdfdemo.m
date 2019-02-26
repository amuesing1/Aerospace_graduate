%Lec13_mvnpdfdemo.m
clc,clear,close all

%define mesh grid of x1 and x2 values for pdf plot
[X1,X2] = meshgrid(-5:0.05:5);
Xgrid = [X1(:),X2(:)];

%%define mean vectors and covariance matrices for different Gaussian pdfs
m1 = [0,0];
C1 = eye(2);

m2 = [1,2];
C2 = [2 0.6; 
      0.6 4];
  
m3 = [-2,3];  
C3=  [2 -1.8;
     -1.8 4];

%%calculate different pdf values on mesh grid
pxy1 = reshape( mvnpdf(Xgrid,m1,C1) , size(X1)); 

pxy2 = reshape( mvnpdf(Xgrid,m2,C2) , size(X1));

pxy3 = reshape( mvnpdf(Xgrid,m3,C3) , size(X1));

%%plot results as pdf surface "heat map"
figure(),
surf(X1,X2,pxy1,'EdgeColor','none'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14)
colorbar
title('Multivariate Normal PDF with mean m1 and covariance C1')

figure(),
surf(X1,X2,pxy2,'EdgeColor','none'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title('Multivariate Normal PDF with mean m2 and covariance C2')

figure(),
surf(X1,X2,pxy3,'EdgeColor','none'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title('Multivariate Normal PDF with mean m3 and covariance C3')