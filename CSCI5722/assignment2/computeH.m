function [ H ] = computeH( points )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
iterations=nchoosek(1:10,4);
H_vecs=zeros(length(iterations),3,3);
distances=zeros(length(iterations),10);
for j=1:length(iterations)
%     selected=randperm(10,4);
    selected=iterations(j,:);
    
    x1=points(selected(1),1);
    y1=points(selected(1),2);
    x1p=points(selected(1),3);
    y1p=points(selected(1),4);
    x2=points(selected(2),1);
    y2=points(selected(2),2);
    x2p=points(selected(2),3);
    y2p=points(selected(2),4);
    x3=points(selected(3),1);
    y3=points(selected(3),2);
    x3p=points(selected(3),3);
    y3p=points(selected(3),4);
    x4=points(selected(4),1);
    y4=points(selected(4),2);
    x4p=points(selected(4),3);
    y4p=points(selected(4),4);
    
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
        x=points(i,1);
        y=points(i,2);
        p=[x;y;1];
        p_new=H*p;
        p_new=p_new/(p_new(3));
        distances(j,i)=norm([points(i,3);points(i,4)]-p_new(1:2));
    end
end
distances=mean(distances,2);
[~,I]=min(distances);
H=squeeze(H_vecs(I,:,:));
end

