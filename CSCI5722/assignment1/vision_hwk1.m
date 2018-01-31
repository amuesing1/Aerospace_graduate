% This script creates a menu driven application

%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 1
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;

% Display a menu and get a choice
choice = menu('Choose an option', 'Exit Program', 'Load Image', ...
    'Display Image', 'Mean Filter','Guassian Filter','Frosty Filter',...
    'Scale Nearest','Scale Bilinear','Swirl Filter','Famous Me');  % as you develop functions, add buttons for them here
 
% Choice 1 is to exit the program
while choice ~= 1
   switch choice
       case 0
           disp('Error - please choose one of the options.')
           % Display a menu and get a choice
           choice = menu('Choose an option', 'Exit Program', 'Load Image', ...
    'Display Image', 'Mean Filter');  % as you develop functions, add buttons for them here
        case 2
           % Load an image
           image_choice = menu('Choose an image', 'lena1', 'mandril1', 'sully', 'yoda');
           switch image_choice
               case 1
                   filename = 'lena1.jpg';
               case 2
                   filename='mandrill1.jpg';
               case 3
                   filename='sully.bmp';
               case 4
                   filename = 'yoda.bmp';
               % fill in cases for all the images you plan to use
           end
           current_img = imread(filename);
       case 3
           % Display image
           figure
           imagesc(current_img);
           if size(current_img,3) == 1
               colormap gray
           end
           
       case 4
           % Mean Filter
           % 1. Ask the user for size of kernel
           k_size = input('What is the kernel size? \n');
           % check that the input is an integer
           if isnumeric(k_size)==1 && rem(k_size,1)==0 && k_size>0
                   
               % 2. Call the appropriate function
               newImage = meanFilter(current_img, k_size); % create your own function for the mean filter
           else
               disp('Error - I need positive integer values.')
           end
           
           % 3. Display the old and the new image using subplot
           % ....
           subplot(1,2,1)
           imagesc(current_img)
           
           subplot(1,2,2)
           imagesc(newImage)
           
           
           % 4. Save the newImage to a file
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_meanfilter','.',og_name(2)));
           imwrite(newImage,new_name);
              
       case 5
           sigma = input('What is the sigma size? \n');
           if isnumeric(sigma)==1 && sigma>0
               newImage = gaussFilter(current_img, sigma);
           else
               disp('Error - I need positive numeric values.')
           end
           
           subplot(1,2,1)
           imagesc(current_img)
           
           subplot(1,2,2)
           imagesc(newImage)
           
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_gaussfilter','.',og_name(2)));
           imwrite(newImage,new_name);
           
       case 6
           n = input('Give me a n! \n');
           m = input('Give me a m! \n');
           if isnumeric(n)==1 && rem(n,1)==0 && n>0 && isnumeric(m)==1 && rem(m,1)==0 && m>0
               newImage = frosty(current_img, n, m);
           else
               disp('Error - I need positive integer values.')
           end
           
           subplot(1,2,1)
           imagesc(current_img)
           
           subplot(1,2,2)
           imagesc(newImage)
           
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_frosty','.',og_name(2)));
           imwrite(newImage,new_name);
           
       case 7
           factor = input('What is the scaling factor? \n');
           if isnumeric(factor)==1 && factor>0
               newImage = scaleNearest(current_img, factor);
           else
               disp('Error - I need positive numeric values.')
           end
           
           subplot(1,2,1)
           imagesc(current_img)
           
           subplot(1,2,2)
           imagesc(newImage)
           
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_scalenearest','.',og_name(2)));
           imwrite(newImage,new_name);
           
       case 8
           factor = input('What is the scaling factor? \n');
           if isnumeric(factor)==1 && factor>0
               newImage = scaleBilinear(current_img, factor);
           else
               disp('Error - I need positive numeric values.')
           end
           
           subplot(1,2,1)
           imagesc(current_img)
           
           subplot(1,2,2)
           imagesc(newImage)
           
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_scalebilinear','.',og_name(2)));
           imwrite(newImage,new_name);
           
       case 9
           factor = input('What is the swirl factor? \n');
           display('Pick a point');
           imagesc(current_img);
           [ox,oy] = ginput(1);
           ox=round(ox);
           oy=round(oy);
           if isnumeric(factor)==1
               if isnumeric(ox)==1 && rem(ox,1)==0 && 0<=ox<=size(current_img,1) ...
                       isnumeric(oy)==1 && rem(oy,1)==0 && 0<=oy<=size(current_img,2)
                   newImage = swirlFilter(current_img, factor,ox,oy);
               else
                   disp('Error - I need positive integers between the size of the image.')
               end
           else
               disp('Error - I need numeric values.')
           end
           
           subplot(1,2,1)
           imagesc(current_img)
           
           subplot(1,2,2)
           imagesc(newImage)
           
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_swirlfilter','.',og_name(2)));
           imwrite(newImage,new_name);
       case 10
           % Photograph taken by Luke Burks
           me_img=imread('jeremy.JPG');
           newImage = famousMe(current_img, me_img);
           
           subplot(1,3,1)
           imagesc(current_img)
           
           subplot(1,3,2)
           imagesc(me_img)
           
           subplot(1,3,3)
           imagesc(newImage)
           
           og_name=strsplit(filename,'.');
           new_name=char(strcat(og_name(1),'_famousMe','.',og_name(2)));
           imwrite(newImage,new_name);
   end
   % Display menu again and get user's choice
   choice = menu('Choose an option', 'Exit Program', 'Load Image', ...
    'Display Image', 'Mean Filter','Gaussian Filter','Frosty Filter',...
    'Scale Nearest','Scale Bilinear','Swirl Filter','Famous Me');  % as you develop functions, add buttons for them here
end
