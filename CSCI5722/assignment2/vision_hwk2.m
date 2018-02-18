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
            h=msgbox('Choose a point on the left image, then select its corresponding point in the right image. Do this 10 times');
            waitfor(h);
            points=getPoints(img1,img2,20);
%             points=[436.736760124611	303.713273195876	885.153188180404	325.717139175258;
%                 438.331775700935	321.316365979381	886.745723172629	346.840850515464;
%                 502.132398753894	399.650128865979	958.409797822706	421.653994845361;
%                 325.085669781931	507.028994845361	781.638413685848	538.714561855670;
%                 443.116822429907	558.958118556701	905.856143079316	591.523840206186;
%                 529.247663551402	315.155283505155	980.705287713841	331.878221649485;
%                 518.082554517134	375.885953608247	977.520217729394	395.249355670103;
%                 497.347352024922	353.001932989691	952.039657853810	375.885953608247;
%                 559.552959501558	337.159149484536	1017.33359253499	352.121778350515;
%                 325.085669781931	531.673324742268	781.638413685848	564.239046391753];
            H=computeH(points);
%             warpped=warp1(img1,img2,H);
            % This one looks a little better imo
            warpped=warp1(img2,img1,inv(H),'mosaic');
            subplot(3,2,1)
            imagesc(img2);
            subplot(3,2,2)
            imagesc(img1);
            subplot(3,2,[3,4,5,6]);
            imagesc(warpped);
        case 3
            img1=imread('pond1.JPG');
            img2=imread('pond2.JPG');
            h=msgbox('Choose a point on the left image, then select its corresponding point in the right image. Do this 10 times');
            waitfor(h);
            points=getPoints(img1,img2,20);
%             points=[367.478193146417	256.814432989691	49.6446345256611	331.041237113402;
%                 358.755451713396	280.010309278351	35.9587869362365	358.876288659794;
%                 360.001557632399	267.639175257732	44.6679626749613	346.505154639175;
%                 388.661993769470	302.432989690722	79.5046656298603	383.618556701031;
%                 454.705607476635	378.979381443299	96.9230171073098	473.309278350515;
%                 711.403426791277	378.979381443299	351.977449455677	457.072164948454;
%                 730.095015576324	301.659793814433	412.941679626750	375.113402061856;
%                 746.294392523364	321.762886597938	427.871695178850	391.350515463918;
%                 727.602803738318	320.989690721650	410.453343701400	392.123711340206;
%                 685.235202492212	339.546391752577	371.884136858476	412.226804123711];
            H=computeH(points);
            warpped=warp1(img1,img2,H,'mosaic');
            h=figure(1);
            subplot(3,2,1)
            imagesc(img2);
            subplot(3,2,2)
            imagesc(img1);
            subplot(3,2,[3,4,5,6]);
            imagesc(warpped);
            waitfor(h);
            % second set of images
            img1=imread('symphony1.JPG');
            img2=imread('symphony2.JPG');
            h=msgbox('Choose a point on the left image, then select its corresponding point in the right image. Do this 10 times');
            waitfor(h);
            points=getPoints(img1,img2,20);
%             points=[531.964174454829	436.195876288660	235.025660964230	478.721649484536;
%                 571.839563862928	437.742268041237	272.350699844479	480.268041237113;
%                 317.633956386293	316.350515463918	177.793934681182	360.422680412371;
%                 331.341121495327	426.144329896907	64.5746500777607	467.896907216495;
%                 726.356697819315	463.257731958763	364.419129082426	505.783505154639;
%                 695.204049844237	465.577319587629	314.652410575428	508.876288659794;
%                 455.951713395639	464.030927835052	54.6213063763610	500.371134020619;
%                 756.263239875389	301.659793814433	681.681959564541	339.546391752577;
%                 357.509345794392	348.051546391753	233.781493001555	399.082474226804;
%                 333.833333333333	287.742268041237	249.955676516330	340.319587628866];
            H=computeH(points);
            warpped=warp1(img1,img2,H,'mosaic');
            h=figure(1);
            subplot(3,2,1)
            imagesc(img2);
            subplot(3,2,2)
            imagesc(img1);
            subplot(3,2,[3,4,5,6]);
            imagesc(warpped);
            waitfor(h)
        case 4
            img1=imread('saul.jpg');
            img2=imread('trump.jpg');
            h=msgbox('Select 4 points in this order: Top left billboard, top left new location, top right, bottom left, bottom right');
            waitfor(h);
            points=getPoints(img1,img2,8);
%             points=[87.2289719626167,37.1494845360824,278.898133748056,242.923969072165;
%                 1173.83333333333,37.1494845360824,519.084758942457,242.923969072165;
%                 83.2414330218068,549.314432989691,261.915241057543,386.289948453608;
%                 1211.71495327103,534.469072164948,531.215396578538,385.725515463918];
%             points=round(points);
            img1=img1(min(points(:,2)):max(points(:,2)),min(points(:,1)):max(points(:,1)),:);
            points(:,1:2)=[0,0;size(img1,2),0;0,size(img1,1);size(img1,2),size(img1,1)];
            H=computeH(points);
            billboard=warp1(img1,img2,H,'warp');
            imshow(billboard)
            
            
            
    end
    choice = menu('Choose an option', 'Exit Program', 'Mosaic of HW Images', ...
        'Mosaic of taken Images', 'Warp Image');
end