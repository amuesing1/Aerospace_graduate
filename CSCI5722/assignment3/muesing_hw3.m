%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 3
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clc;

load('handshakeStereoParams.mat');
videoFileLeft = 'handshake_left.avi';
videoFileRight = 'handshake_right.avi';

readerLeft = vision.VideoFileReader(videoFileLeft, 'VideoOutputDataType', 'uint8');
readerRight = vision.VideoFileReader(videoFileRight, 'VideoOutputDataType', 'uint8');

frameLeft = readerLeft.step();
frameRight = readerRight.step();

[frameLeftRect, frameRightRect] = ...
    rectifyStereoImages(frameLeft, frameRight, stereoParams);
frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);

disparityMap = disparity(frameLeftGray, frameRightGray);

% Task 1
% ssd_1=SSD(frameLeftGray,frameRightGray,1,'RL');
% ssd_5=SSD(frameLeftGray,frameRightGray,5,'RL');
% ssd_11=SSD(frameLeftGray,frameRightGray,11,'RL');
% save('task1.mat','ssd_1','ssd_5','ssd_11')
load('task1.mat');
figure(1)
subplot(2,2,1)
imshow(ssd_1,[0,64]);
title('SSD Disparity Map (1x1)');
colorbar
subplot(2,2,2)
imshow(ssd_5,[0,64]);
title('SSD Disparity Map (5x5)');
colorbar
subplot(2,2,3)
imshow(ssd_11,[0,64]);
title('SSD Disparity Map (11x11)');
colorbar
subplot(2,2,4)
imshow(disparityMap,[0,64]);
title('Matlab Disparity Map');
colorbar

% Task 2
% ncc_3=NCC(frameLeftGray,frameRightGray,3,'RL');
% ncc_5=NCC(frameLeftGray,frameRightGray,5,'RL');
% ncc_7=NCC(frameLeftGray,frameRightGray,7,'RL');
% save('task2.mat','ncc_3','ncc_5','ncc_7')
load('task2.mat');
figure(2)
subplot(2,2,1)
imshow(ncc_3,[0,64]);
title('NCC Disparity Map (3x3)');
colorbar
subplot(2,2,2)
imshow(ncc_5,[0,64]);
title('NCC Disparity Map (5x5)');
colorbar
subplot(2,2,3)
imshow(ncc_7,[0,64]);
title('NCC Disparity Map (7x7)');
colorbar
subplot(2,2,4)
imshow(disparityMap,[0,64]);
title('Matlab Disparity Map');
colorbar

% Task 4
% window_size=3;
% d_ssd_lr=unique_SSD(frameLeftGray,frameRightGray,window_size,'LR');
% d_ssd_rl=unique_SSD(frameLeftGray,frameRightGray,window_size,'RL');
% d_ncc_lr=unique_NCC(frameLeftGray,frameRightGray,window_size,'LR');
% d_ncc_rl=unique_NCC(frameLeftGray,frameRightGray,window_size,'RL');
% save('task4.mat','d_ssd_lr','d_ssd_rl','d_ncc_lr','d_ncc_rl');
load('task4.mat');
out_ssd=outliers(d_ssd_lr,d_ssd_rl,1);
out_ncc=outliers(d_ncc_lr,d_ssd_rl,1);
figure(3)
subplot(1,2,1)
imshow(out_ssd,[0,1])
title('SSD Outliers')
subplot(1,2,2)
imshow(out_ncc,[0,1])
title('NCC Outliers')

% Task 5
f=stereoParams.CameraParameters1.FocalLength(1);
B=abs(stereoParams.TranslationOfCamera2(1));
Z=(f*B)./double(d_ssd_rl);
Z(~isfinite(Z))=0;
figure(4)
imshow(Z,[0,max(max(Z))])
title('Depth from Disparity')

% Task 6
% left=imread('frame_1L.png');
left_true=imread('frame_1LR.png');
% left=rgb2gray(left);
% right=imread('frame_1R.png');
right_true=imread('frame_1RL.png');
% right=rgb2gray(right);
% 
% d_ssd_lr_6=unique_SSD(left,right,window_size,'LR');
% d_ssd_rl_6=unique_SSD(left,right,window_size,'RL');
% save('task6.mat','d_ssd_lr_6','d_ssd_rl_6');
load('task6.mat');
diff_lr=left_true-d_ssd_lr_6;
diff_rl=right_true-d_ssd_rl_6;
figure(5)
subplot(1,2,1)
imshow(diff_lr,[0,64])
colorbar
title('LR errors')
subplot(1,2,2)
imshow(diff_rl,[0,64])
colorbar
title('RL errors')

% figure(6)
subplot(1,2,1)
histogram(diff_lr)
title('LR Errors')
xlabel('Size of Error')
subplot(1,2,2)
histogram(diff_rl)
title('RL Errors')
xlabel('Size of Error')

% Task 7
% occ=0.01;
% maxDisp=63;
% dis=zeros(size(frameLeftGray));
% for i=1:size(frameLeftGray,1)
%     e1=frameLeftGray(i,:);
%     e2=frameRightGray(i,:);
%     dis(i,:)=stereoDP(e1,e2,maxDisp,occ);
% end
save('task7.mat','dis');
load('task7.mat');
max_d=max(max(dis));
min_d=min(min(dis));
dis=(dis-min_d)./(max_d-min_d);
reddis=dis;
reddis(isnan(reddis))=1;
new=zeros(size(dis,1),size(dis,2),3);
new(:,:,1)=reddis;
new(:,:,2)=dis;
new(:,:,3)=dis;
figure(7)
imshow(new);
title('Dynamic Programming and Occlusion');
