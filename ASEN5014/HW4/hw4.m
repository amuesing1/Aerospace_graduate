clear all; clc; close all;

A=[3 1 0;-1 1 0;3 2 2];
M=[0 1 -2; 0 -1 3; 1 0 0];
inv(M)*A*M
% inv(M)*A*M
% A*M
[vec,val]=eig(A)
% expm(A)