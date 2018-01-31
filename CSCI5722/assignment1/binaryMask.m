% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming
function [ outImg ] = binaryMask( inImg)
% Creates an image mask matrix from an image with a consistent background

threshold=1500;
dimentions=size(inImg);
outImg=zeros(dimentions(1),dimentions(2));

imshow(inImg);
[x,y,button]=ginput(1);

for i=1:dimentions(1)
    for j=1:dimentions(2)
        % convert to greyscale without using the image proccessing toolbox
        outImg(i,j)=0.299*inImg(i,j,1)+0.587*inImg(i,j,2)+0.114*inImg(i,j,3);
    end
end

% background=mode(mode(outImg));
background=outImg(round(x),round(y));
for i=1:dimentions(1)
    for j=1:dimentions(2)
        % checking how different the pixels are from the background
        if (outImg(i,j)-background)^2 > threshold
            outImg(i,j)=1;
        else
            outImg(i,j)=0;
        end
    end
end

end

