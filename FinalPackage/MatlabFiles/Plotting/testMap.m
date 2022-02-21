clear
clc
close all

load('RALmap.txt')
load('RALnodes.txt')
ratio = 1;
xScale = 1.1217;
yScale = 0.9143;
xScale = 1;
yScale = 1;

RALmap(:,1) = RALmap(:,1)*xScale;
RALmap(:,3) = RALmap(:,3)*xScale;
RALmap(:,2) = RALmap(:,2)*yScale;
RALmap(:,4) = RALmap(:,4)*yScale;

xwall=[RALmap(:,1) RALmap(:,3)];
ywall=[RALmap(:,2) RALmap(:,4)];
writematrix(RALmap, 'RALMapScaled.txt','Delimiter','space')

RALnodes(:,1) = RALnodes(:,1)*xScale;
RALnodes(:,2) = RALnodes(:,2)*yScale;
% plot(RALnodes(:,1),RALnodes(:,2),'bo')
writematrix(RALnodes, 'RALNodesScaled.txt','Delimiter','space')
rectangle('Position',[-1.66,1.2,-.8,1.5],'FaceColor',[0 0 0])
rectangle('Position',[-1.66,0.6,-0.8,.9],'FaceColor',[0 0 0])
rectangle('Position',[-1.75,-.25,-.75,0],'FaceColor',[0 0 0])
rectangle('Position',[-.2,-.75,.1,.5],'FaceColor',[0 0 0])
% rectangle('Position',[8,-5,24,7],'FaceColor',[0 0 0])
% rectangle('Position',[11,6,4,3],'FaceColor',[0 0 0])
hold on 
plot(xwall',ywall','k-','linewidth',1.2);
plot(0,0,'r.','markersize',60);
% plot(-.6,1,'b^','markerfacecolor','b');
% plot(-.6,-.4,'b^','markerfacecolor','b');
% plot(-.3,-1,'gs','markerfacecolor','g');
% plot(.3,-1,'gs','markerfacecolor','g');
hold off
xOffset = .1;
yOffset = .21;
xlim([min([RALmap(:,1);RALmap(:,3)])-.05, max([RALmap(:,1);RALmap(:,3)])+.05+xOffset])
ylim([min([RALmap(:,2);RALmap(:,4)])-.05-yOffset, max([RALmap(:,2);RALmap(:,4)])+.05])

%set(gca,'xdir','reverse','ydir','reverse')
set(gca,'DataAspectRatioMode','auto')

set(gca,'Position',[0 0 1 1])
%set(gcf,'MenuBar','none')
set(gca,'XTick',[])
set(gca,'YTick',[])
% saveas(gcf,'workspace.png')
%imwrite(getframe(gca).cdata, 'workspace.png')
