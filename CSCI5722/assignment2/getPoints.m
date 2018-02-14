%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 2
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ corresponding_array ] = getPoints(img1,img2,N )
% Grabs points from two figures to be transformed

% displaying figures
hold on;
figure('units','normalized','outerposition',[0 0 1 1])
subplot(1,2,1)
imagesc(img1);
subplot(1,2,2)
imagesc(img2);
corresponding_array=zeros(N/2,4);
for i=1:N
    [x,y]=ginput(1);
    hold on;
    % plots the point you selected
    plot(x,y,'ro')
    x_ref=ceil(i/2);
    if rem(i,2)==1
        y_ref=1;
    else
        y_ref=3;
    end
    % setting points in array
    corresponding_array(x_ref,y_ref:y_ref+1)=x;
    corresponding_array(x_ref,y_ref+1)=y;
end
close all;

end

