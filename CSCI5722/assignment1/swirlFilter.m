% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming

% Found the rotation equation here
% https://angeljohnsy.blogspot.com/2011/06/swirl-effect-in-matlab.html

function [ outImg ] = swirlFilter( inImg, factor,ox,oy )
% Swirls an image by a user specified amount about a user specified point
% with points further out being rotated more

dimentions=size(inImg);

x_ref=zeros(dimentions(1),dimentions(2));
y_ref=zeros(dimentions(1),dimentions(2));
for i=1:dimentions(1)
    x=i-ox;
    for j=1:dimentions(2)
        %Finding the current rotation and distance from center in Polar
        [theta,rho]=cart2pol(x,j-oy);
        
        %Rotation to new angle from center
        theta_new=theta+(factor*rho/500); %while this rotates nicely, the 
                                    %magnitude is super high. Want users
                                    %to enter a resonable integer
        
        %Placing pixel in new location
        [l,m]=pol2cart(theta_new,rho);
        x_ref(i,j)=round(l)+ox;
        y_ref(i,j)=round(m)+oy;
        
    end
end
%Make sure indicies are inside the original image
x_ref=max(x_ref,1);
x_ref=min(x_ref,dimentions(1));

y_ref=max(y_ref,1);
y_ref=min(y_ref,dimentions(2));

new_image=zeros(dimentions);
for i=1:dimentions(1)
    for j=1:dimentions(2)
        new_image(i,j,:)=inImg(x_ref(i,j),y_ref(i,j),:);
    end
end
outImg=uint8(new_image);
end

