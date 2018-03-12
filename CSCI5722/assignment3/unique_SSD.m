%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 3
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ D ] = unique_SSD( img1,img2,window_size,direction )
% Computes the disparity map using the ssd algorithm. This also only allows
% one match per pixel.The rule used to decide between similar matches is 
% the to take the closest one
if window_size>1
    offset=floor(window_size/2);
    sigma=sqrt(window_size)/2;
    guassian=ones(offset*2+1,offset*2+1);
    [X,Y]=meshgrid(-offset:offset,-offset:offset);
    for i=1:size(X,1)
        for j=1:size(X,2)
            guassian(i,j)=mvnpdf([X(i,j),Y(i,j)],[0,0],[sigma,0;0,sigma]);
        end
    end
    guassian=guassian./sum(sum(guassian));
else
    offset=0;
    guassian=1;
end
img1=double(img1);
img2=double(img2);
D=zeros(size(img1));
for i=(1+offset):(size(img1,1)-offset)
    for j=(1+offset):(size(img1,2)-offset)
        k_used=[];
        min=255^2;
        min_index=0;
        if direction=='LR'
            if j+63+offset>=size(img2,2)
                max_range=size(img2,2)-j-offset;
            else
                max_range=63;
            end
            img1_vec=guassian.*img1(i-offset:i+offset,j-offset:j+offset);
            for k=j:j+max_range
                if ismember(k,k_used)
                    continue
                else
                    diff=img1_vec-guassian.*img2(i-offset:i+offset,k-offset:k+offset);
                    SSD=sum(diff.^2);
                    % This ensures that the first and closest match is always
                % chosen and kept. That is my criteria for dealing with
                % multiple matches
                    if SSD<min
                        min=SSD;
                        min_index=k-j;
                    end
                end
            end
            D(i,j)=min_index;
            k_used=[k_used,min_index+j];
        elseif direction=='RL'
            if j-63-offset<=0
                max_range=j-offset-1;
            else
                max_range=63;
            end
            img1_vec=guassian.*img1(i-offset:i+offset,j-offset:j+offset);
            for k=j:-1:j-max_range
                if ismember(k,k_used)
                    continue
                else
                    diff=img1_vec-guassian.*img2(i-offset:i+offset,k-offset:k+offset);
                    SSD=sum(sum(diff.^2));
                    if SSD<min
                        min=SSD;
                        min_index=j-k;
                    end
                end
            end
            D(i,j)=min_index;
            k_used=[k_used,j-min_index];
        end
    end
end
D=uint8(D);
end

