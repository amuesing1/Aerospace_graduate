% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming

% Found the bilnear equation and sub2ind function here 
% https://stackoverflow.com/questions/26142288/resize-an-image-with-bilinear-interpolation-without-imresize

function [ outImg ] = scaleBilinear( inImg, factor )
% Resizes an image using the Bilinear equation. 

dimentions=size(inImg);

rows_i = dimentions(1);
cols_i = dimentions(2);
rows_f = dimentions(1)*factor;
cols_f = dimentions(2)*factor;

% We need this because bilinear does both row and column calculations      
S_R = 1/factor;
S_C = 1/factor;

[cf,rf]=meshgrid(1 : cols_f, 1 : rows_f);

rf = rf * S_R;
cf = cf * S_C;

r = floor(rf);
c = floor(cf);

%Make sure there are no indexes that aren't in original image
r(r < 1) = 1;
c(c < 1) = 1;
r(r > rows_i - 1) = rows_i - 1;
c(c > cols_i - 1) = cols_i - 1;

delta_R = rf - r;
delta_C = cf - c;

% This becomes a nightmare to do with for loops so we grab column major
% indicies for each of the four points that need to be sampled
I_sample_norm = sub2ind([rows_i, cols_i], r, c);
I_sample_down = sub2ind([rows_i, cols_i], r+1,c);
I_sample_right = sub2ind([rows_i, cols_i], r, c+1);
I_sample_down_right = sub2ind([rows_i, cols_i], r+1, c+1);     

new_image=zeros(floor(dimentions(1)*factor),floor(dimentions(2)*factor),dimentions(3));

for k = 1 : dimentions(3)
    layer = double(inImg(:,:,k));
    new_image(:,:,k) = layer(I_sample_norm).*(1 - delta_R).*(1 - delta_C) + ...
                   layer(I_sample_down).*(delta_R).*(1 - delta_C) + ...
                   layer(I_sample_right).*(1 - delta_R).*(delta_C) + ...
                   layer(I_sample_down_right).*(delta_R).*(delta_C);
end
outImg=uint8(new_image);
end