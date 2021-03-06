% Jeremy Muesing
% CSCI 5722
% Assignment 4
% Instructor: Dr. Ioana Fleming
function featuresNorm = NormalizeFeatures(features)
% Normalize image features to have zero mean and unit variance. This
% normalization can cause k-means clustering to perform better.
%
% INPUTS
% features - An array of features for an image. features(i, j, :) is the
%            feature vector for the pixel img(i, j, :) of the original
%            image.
%
% OUTPUTS
% featuresNorm - An array of the same shape as features where each feature
%                has been normalized to have zero mean and unit variance.

    features = double(features);
    featuresNorm = features;
    s=size(features);
    tempfeat=reshape(features,s(1)*s(2),s(3));
    means=mean(tempfeat);
    stds=std(tempfeat);
    teampfeat=(tempfeat-means)./stds;
    featuresNorm=reshape(teampfeat,s(1),s(2),s(3));
end