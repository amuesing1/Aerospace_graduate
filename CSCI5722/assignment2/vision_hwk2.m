%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 2
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;close all;clc;

% Display a menu and get a choice
choice = menu('Choose an option', 'Exit Program', 'Mosaic of HW Images', ...
    'Mosaic of taken Images', 'Warp Image');
while choice ~= 1
    switch choice
        case 0
            disp('Error - please choose one of the options.');
            choice = menu('Choose an option', 'Exit Program', 'Mosaic of HW Images', ...
                'Mosaic of taken Images', 'Warp Image');
        case 2
            img1=imread('uttower1.JPG');
            img2=imread('uttower2.JPG');
            %points=getPoints(img1,img2);
            points=[436.736760124611	303.713273195876	885.153188180404	325.717139175258;
                438.331775700935	321.316365979381	886.745723172629	346.840850515464;
                502.132398753894	399.650128865979	958.409797822706	421.653994845361;
                325.085669781931	507.028994845361	781.638413685848	538.714561855670;
                443.116822429907	558.958118556701	905.856143079316	591.523840206186;
                529.247663551402	315.155283505155	980.705287713841	331.878221649485;
                518.082554517134	375.885953608247	977.520217729394	395.249355670103;
                497.347352024922	353.001932989691	952.039657853810	375.885953608247;
                559.552959501558	337.159149484536	1017.33359253499	352.121778350515;
                325.085669781931	531.673324742268	781.638413685848	564.239046391753];
            H=computeH(points);
            warpped=warp1(img1,img2,H);
    end
    choice = menu('Choose an option', 'Exit Program', 'Mosaic of HW Images', ...
        'Mosaic of taken Images', 'Warp Image');
end