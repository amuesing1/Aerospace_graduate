% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming
function [ outImg ] = scaleNearest( inImg, factor )
% Scales an image based off of the nearest neighbor pixel

dimentions=size(inImg);
new_image=zeros(floor(dimentions(1)*factor),floor(dimentions(2)*factor),dimentions(3));

for i=1:floor(dimentions(1)*factor)
    for j=1:floor(dimentions(2)*factor)
        x=i/factor;
        y=j/factor;
        x=max(1,round(x));
        y=max(1,round(y));
        new_image(i,j,1:3)=inImg(x,y,1:3);
    end
end
outImg=uint8(new_image);
end

