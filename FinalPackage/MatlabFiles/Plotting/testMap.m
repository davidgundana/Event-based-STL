clear
clc
close all

load('RALmap.txt')
load('RALnodes.txt')
ratio = 1.4;
xScale = 2.18/2;
yScale = xScale/ratio;

RALmap(:,1) = RALmap(:,1)*xScale;
RALmap(:,3) = RALmap(:,3)*xScale;
RALmap(:,2) = RALmap(:,2)*yScale;
RALmap(:,4) = RALmap(:,4)*yScale;

xwall=[RALmap(:,1) RALmap(:,3)];
ywall=[RALmap(:,2) RALmap(:,4)];
writematrix(RALmap, 'RALMapScaled.txt','Delimiter','space')

RALnodes(:,1) = RALnodes(:,1)*xScale;
RALnodes(:,2) = RALnodes(:,2)*yScale;
plot(RALnodes(:,1),RALnodes(:,2),'bo')
writematrix(RALnodes, 'RALNodesScaled.txt','Delimiter','space')
% rectangle('Position',[-5,30,50,10],'FaceColor',[0 0 0])
% rectangle('Position',[-5,-10,50,10],'FaceColor',[0 0 0])
% rectangle('Position',[-5,-5,5,40],'FaceColor',[0 0 0])
% rectangle('Position',[40,-5,5,50],'FaceColor',[0 0 0])
% rectangle('Position',[8,-5,24,7],'FaceColor',[0 0 0])
% rectangle('Position',[11,6,4,3],'FaceColor',[0 0 0])
hold on 
plot(xwall',ywall','k-','linewidth',1.2);
plot(0,0,'r.','markersize',30);
% plot(-.6,1,'b^','markerfacecolor','b');
% plot(-.6,-.4,'b^','markerfacecolor','b');
% plot(-.3,-1,'gs','markerfacecolor','g');
% plot(.3,-1,'gs','markerfacecolor','g');
hold off
axis equal
xOffset = .1;
yOffset = .3;
xlim([min([RALmap(:,1);RALmap(:,3)])-.1, max([RALmap(:,1);RALmap(:,3)])+.1+xOffset])
ylim([min([RALmap(:,2);RALmap(:,4)])-yOffset, max([RALmap(:,2);RALmap(:,4)])+.1])
% saveas(gcf,'workspace.png')
imwrite(getframe(gca).cdata, 'workspace.png')
