%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Jeremy Muesing
% CSCI 5722
% Assignment 2
% Instructor: Fleming
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ H ] = computeH( points )
% This function will find the homography matrix with the smallest distance
% transformation from any number of points. The smallest number of points
% must be 4

% Math for homography matric found here
%https://math.stackexchange.com/questions/494238/how-to-compute-homography-matrix-h-from-corresponding-points-2d-2d-planar-homog
iterations=nchoosek(1:length(points),4);

% if there are only 4 points, there is only one variation that can be
% computed
if size(iterations,1)==1
    x1=points(1,2);
    y1=points(1,1);
    x1p=points(1,4);
    y1p=points(1,3);
    x2=points(2,2);
    y2=points(2,1);
    x2p=points(2,4);
    y2p=points(2,3);
    x3=points(3,2);
    y3=points(3,1);
    x3p=points(3,4);
    y3p=points(3,3);
    x4=points(4,2);
    y4=points(4,1);
    x4p=points(4,4);
    y4p=points(4,3);
    
    P=[-x1,-y1,-1,0,0,0,x1*x1p,y1*x1p,x1p;
        0,0,0,-x1,-y1,-1,x1*y1p,y1*y1p,y1p;
        -x2,-y2,-1,0,0,0,x2*x2p,y2*x2p,x2p;
        0,0,0,-x2,-y2,-1,x2*y2p,y2*y2p,y2p;
        -x3,-y3,-1,0,0,0,x3*x3p,y3*x3p,x3p;
        0,0,0,-x3,-y3,-1,x3*y3p,y3*y3p,y3p;
        -x4,-y4,-1,0,0,0,x4*x4p,y4*x4p,x4p;
        0,0,0,-x4,-y4,-1,x4*y4p,y4*y4p,y4p];
    
    [~,~,V]=svd(P);
    H=vec2mat(V(:,end),3);
    
else
    % if there are more than 4 points, there will be a variation with the
    % smallest transform distance. The following loop finds that.
    H_vecs=zeros(length(iterations),3,3);
    distances=zeros(length(iterations),length(points));
    for j=1:length(iterations)
        selected=iterations(j,:);
        
        % ginput grabs x,y coordinates that are switched for row and column
        % from matlab's default indexing. This both assigns the points and
        % switches the axes.
        x1=points(selected(1),2);
        y1=points(selected(1),1);
        x1p=points(selected(1),4);
        y1p=points(selected(1),3);
        x2=points(selected(2),2);
        y2=points(selected(2),1);
        x2p=points(selected(2),4);
        y2p=points(selected(2),3);
        x3=points(selected(3),2);
        y3=points(selected(3),1);
        x3p=points(selected(3),4);
        y3p=points(selected(3),3);
        x4=points(selected(4),2);
        y4=points(selected(4),1);
        x4p=points(selected(4),4);
        y4p=points(selected(4),3);
        
        P=[-x1,-y1,-1,0,0,0,x1*x1p,y1*x1p,x1p;
            0,0,0,-x1,-y1,-1,x1*y1p,y1*y1p,y1p;
            -x2,-y2,-1,0,0,0,x2*x2p,y2*x2p,x2p;
            0,0,0,-x2,-y2,-1,x2*y2p,y2*y2p,y2p;
            -x3,-y3,-1,0,0,0,x3*x3p,y3*x3p,x3p;
            0,0,0,-x3,-y3,-1,x3*y3p,y3*y3p,y3p;
            -x4,-y4,-1,0,0,0,x4*x4p,y4*x4p,x4p;
            0,0,0,-x4,-y4,-1,x4*y4p,y4*y4p,y4p];
        
        [~,~,V]=svd(P);
        H=vec2mat(V(:,end),3);
        H_vecs(j,:,:)=H;
        
        for i=1:length(points)
            x=points(i,2);
            y=points(i,1);
            p=[x;y;1];
            p_new=H*p;
            p_new=p_new/(p_new(3));
            distances(j,i)=norm([points(i,4);points(i,3)]-p_new(1:2));
        end
    end
    distances=mean(distances,2);
    [~,I]=min(distances);
    H=squeeze(H_vecs(I,:,:));
end
end

