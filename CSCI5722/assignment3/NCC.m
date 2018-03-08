%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 3
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ D ] = NCC( img1,img2,window_size,direction )
% Computes the disparity map using the ncc algorithm. The rule used to
% decide between similar matches is the to take the closest one
offset=floor(window_size/2);
img1=double(img1);
img2=double(img2);
D=zeros(size(img1));
for i=1:size(img1,1)
    for j=(1+offset):(size(img1,2)-offset)
        min=255^2;
        min_index=0;
        if direction=='LR'
            if j+63+offset>=size(img2,2)
                max_range=size(img2,2)-j-offset;
            else
                max_range=63;
            end
            img1_vec=img1(i,j-offset:j+offset)/norm(img1(i,j-offset:j+offset));
            for k=j:j+max_range
                NCC=dot(img1_vec,img2(i,k-offset:k+offset)/norm(img2(i,k-offset:k+offset)))/window_size;
                if NCC<min
                    min=NCC;
                    min_index=k-j;
                end
            end
            D(i,j)=min_index;
        elseif direction=='RL'
            if j-63-offset<=0
                max_range=j-offset-1;
            else
                max_range=63;
            end
            img1_vec=img1(i,j-offset:j+offset)/norm(img1(i,j-offset:j+offset));
            for k=j:-1:j-max_range
                NCC=dot(img1_vec,img2(i,k-offset:k+offset)/norm(img2(i,k-offset:k+offset)))/window_size;
                if NCC<min
                    min=NCC;
                    min_index=j-k;
                end
            end
            D(i,j)=min_index;
        end
    end
end
D=uint8(D);
end