function [ new_image ] = warp1( img1,img2,H )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% figure out the bounding box
corners=[0 0 size(img1,1) size(img1,1); 0 size(img1,2) 0 size(img1,2); 1 1 1 1];
transformed_corners=H*corners;
normalize = transformed_corners(3,:);
transformed_corners= [transformed_corners(1,:)./normalize; transformed_corners(2,:)./normalize];
min_max_array=[transformed_corners,[0,size(img2,1);0,size(img2,2)]];
x_min=min(min_max_array(1,:));
x_max=max(min_max_array(1,:));
y_min=min(min_max_array(2,:));
y_max=max(min_max_array(2,:));
x_size=x_max-x_min;
y_size=y_max-y_min;
% shift the static image
new_image=NaN(ceil(x_size),ceil(y_size),3);
x_shift=round(abs(x_min));
y_shift=round(abs(y_min));
for i=1:size(img2,1)
    for j=1:size(img2,2)
        new_image(i+x_shift,j+y_shift,:)=img2(i,j,:);
    end
end
imshow(uint8(new_image))
coordinates_new=ones(3,size(new_image,1)*size(new_image,2));
count=1;
for i=1:size(new_image,1)
    for j=1:size(new_image,2)
        coordinates_new(1:2,count)=[i,j];
        count=count+1;
    end
end
old_points = inv(H) * coordinates_new;
normalize = old_points(3,:);
final_points_old = [old_points(1,:)./normalize; old_points(2,:)./normalize];
for i=1:length(coordinates_new)
    if final_points_old(1,i)<1 || final_points_old(2,i)<1 || final_points_old(1,i)>size(img1,1) || final_points_old(2,i)>size(img1,2)
        continue
    else
        for k=1:3
            new_image(coordinates_new(1,i),coordinates_new(2,i),k)=img1(round(final_points_old(1,i)),round(final_points_old(2,i)),k);
        end
    end
end
imshow(uint8(new_image))
end

