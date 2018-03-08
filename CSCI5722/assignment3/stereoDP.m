%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 3
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ disparity ] = stereoDP( Il,Ir,maxDisp,occ )
% Dynamic Programming function to find the disparity of a single equipolar
% line
D=zeros(length(Il),length(Ir));
dir=zeros(length(Il),length(Ir));
disparity=NaN(1,length(Il));
Il=double(Il);
Ir=double(Ir);
D(1,1)=(Il(1)-Ir(1))^2;
% create the D and direction matricies
for i=2:length(Il)
    for j=2:length(Ir)
        D(1,j)=j*occ;
        D(i,1)=i*occ;
        dir(i,1)=-1;
        dir(1,j)=1;
        if (D(i-1,j-1)+(Il(i)-Ir(j))^2)<(D(i-1,j)+occ) && (D(i-1,j-1)+(Il(i)-Ir(j))^2)<(D(i,j-1)+occ)
            dir(i,j)=0; %nw
        elseif (D(i-1,j)+occ)<(D(i-1,j-1)+(Il(i)-Ir(j))^2) && (D(i-1,j)+occ)<(D(i,j-1)+occ)
            dir(i,j)=1; %n
        elseif (D(i,j-1)+occ)<(D(i-1,j)+occ) && (D(i,j-1)+occ)<(D(i-1,j-1)+(Il(i)-Ir(j))^2)
            dir(i,j)=-1; %w
        end
        D(i,j)=min([D(i-1,j-1)+(Il(i)-Ir(j))^2,D(i-1,j)+occ,D(i,j-1)+occ]);
    end
end
% move through the array and compute disparity
i=length(Il);
j=length(Ir);
while i~=1 && j~=1
    if dir(i,j)==0
        disparity(i)=min(i-j,maxDisp);
        i=i-1;
        j=j-1;
    elseif dir(i,j)==(-1)
        j=j-1;
    elseif dir(i,j)==1
        i=i-1;
    end
end
end

