% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming
function [ outImg ] = gaussFilter( inImg,sigma )
% Applies a user specified amount of guassian blur to an image

kernel_size=2*ceil(2*sigma)+1;

dimentions=size(inImg);
if rem(kernel_size,2)==1
    padding=floor(kernel_size/2);
new_image=padarray(inImg,[padding,padding],NaN);

for i=1:dimentions(1)
    for j=1:dimentions(2)
        for k=1:dimentions(3)
            G_array=zeros(kernel_size,kernel_size);
            for l=-padding:padding
                for m=-padding:padding
                    % guass filter equation
                    G=(1/(2*pi*sigma^2))*exp(-(l^2+m^2)/(2*sigma^2));
                    G_array(m+1+padding,l+1+padding)=G;
                end
            end
            G_array=G_array/sum(G_array(:));
            A=sum(sum(G_array.*double(new_image(i:(i+2*padding),j:(j+2*padding),k))));
            inImg(i,j,k)=A;
        end
    end
end
outImg=inImg;
end

