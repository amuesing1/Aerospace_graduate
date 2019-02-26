%resampleParticles.m

function [sampsOut,wtsOut] = resampleParticles(sampsIn,wtsIn)
nsampsIn = size(sampsIn,2);
sampcdf = repmat(cumsum(wtsIn)',[1 nsampsIn]);

%draw a bunch of random #'s from U(0,1):
urands = repmat(rand(1,nsampsIn),nsampsIn,1);

%compare tomcruise to CDF:
tag = (urands<sampcdf); %returns a logical array
tag = single(tag); %convert to a numeric array
tagsums = cumsum(tag,1); %tally up # of times tomcruise is less than cdf

%for each sample (column) extract the rows where the first < occurs in tag
[indsampsout,~] = find(tagsums==1);
sampsOut = sampsIn(:,indsampsout);
wtsOut = 1/nsampsIn*ones(size(wtsIn));
