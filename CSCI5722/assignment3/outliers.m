%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 3
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ D ] = outliers( d_LR,d_RL,T_LR )
% Finds the outliers for any given sets of LR and RL stereo images
D=zeros(size(d_LR));
for i=1:size(d_LR,1)
    for j=1:size(d_LR,2)
        if abs(d_LR(i,j)-d_RL(i,max(j-d_LR(i,j),1)))<=T_LR
            D(i,j)=0;
        else
            D(i,j)=1;
        end
    end
end
D=uint8(D);
end

