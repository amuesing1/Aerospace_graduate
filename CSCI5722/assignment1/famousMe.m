% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming
function [ outImg ] = famousMe( inImg, me_img )
% Places one image on top of another. The me_img must have a uniform
% background.

% scaling
dimentions=size(inImg);
dimentions2=size(me_img);
factor=min(dimentions(1),dimentions(2))/max(dimentions2(1),dimentions2(2));
me_img=scaleBilinear(me_img,factor);
dimentions2=size(me_img);

masked_pixels=binaryMask(me_img);
startx=floor(dimentions(1)/2)-floor(dimentions2(1)/2);
starty=floor(dimentions(2)/2)-floor(dimentions2(2)/2);

outImg=inImg;

% replacing pixels
for i=1:dimentions2(1)
    for j=1:dimentions2(2)
        if masked_pixels(i,j)==1
            for k=1:3
                outImg(i+startx,j+starty,k)=me_img(i,j,k);
            end
        end
    end
end

end

