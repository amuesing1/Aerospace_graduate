% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming
function [ outImg ] = meanFilter( inImg,kernel_size )
% Applies a user specified amount of blur by averaging the neighboring
% pixels

dimentions=size(inImg);
padding=floor(kernel_size/2);
new_image=padarray(double(inImg),[padding,padding],NaN);

for i=1:dimentions(1)
    for j=1:dimentions(2)
        for k=1:dimentions(3)
            inImg(i,j,k)=nanmean(nanmean(new_image(i:(i+2*padding),j:(j+2*padding),k)));
        end
    end
end
outImg=inImg;
end

