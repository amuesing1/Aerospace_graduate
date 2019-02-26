% Jeremy Muesing
% CSCI 5722
% Assignment 4
% Instructor: Dr. Ioana Fleming
function features = ComputeFeatures(img)
% Compute a feature vector for all pixels of an image. You can use this
% function as a starting point to implement your own custom feature
% vectors.
%
% INPUT
% img - Array of image data of size h x w x 3.
%
% OUTPUT
% features - Array of computed features for all pixels of size h x w x f
%            such that features(i, j, :) is the feature vector (of
%            dimension f) for the pixel img(i, j, :).

    img = double(img);
    height = size(img, 1);
    width = size(img, 2);
    features = zeros([height, width, 3]);

% Computes the entropy value of each pixel for the corresponding
% neighborhood in the surround area
    new_image=entropyfilt(rgb2gray(img));
    features(:,:,1)=new_image;
    features(:,:,2)=repmat([1:width],height,1);
    features(:,:,3)=repmat([1:height]',1,width);
end