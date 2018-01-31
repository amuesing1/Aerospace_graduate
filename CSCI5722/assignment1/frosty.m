% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming

% Worked with Luke Burks to develop the non-for loop version of the
% algorithm

function [ outImg ] = frosty( inImg,n,m )
% Applies a frosty filter to an image. The amount of frost is specified by
% the user. This is also done WITHOUT FOR LOOPS.

dimentions=size(inImg);
xpad=floor(n/2);
ypad=floor(m/2);

new_image=padarray(double(inImg),[floor(n/2),floor(m/2)],NaN);

% create indicies that will be changed for each channel
[x_change,y_change]=meshgrid(1:dimentions(2),1:dimentions(1));
%randomly generate pixel distances to change
randX=randi([-xpad,xpad],dimentions(1),dimentions(2));
randY=randi([-ypad,ypad],dimentions(1),dimentions(2));
% add the changes to the indicies
x_change=min(dimentions(2),max(n,x_change+randX+xpad));
y_change=min(dimentions(1),max(m,y_change+randY+ypad));
%make sure no indicies are larger than image
x_change=min(x_change,size(new_image,2));
y_change=min(y_change,size(new_image,1));
% split up the channels because we will be doing this via column major
% indicies
new_image_1=new_image(:,:,1);
new_image_2=new_image(:,:,2);
new_image_3=new_image(:,:,3);
% get the column major indicies of the moved pixels
change_total=sub2ind(size(new_image_1),y_change,x_change);
% set each channel to the values in the moved indicies
new_image_1=new_image_1(change_total);
new_image_2=new_image_2(change_total);
new_image_3=new_image_3(change_total);
% combine the channels back together
outImg=uint8(cat(3,new_image_1,new_image_2,new_image_3));
end

