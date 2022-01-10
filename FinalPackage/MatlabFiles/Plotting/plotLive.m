function [rob,robAng] = plotLive(x,y,theta,posPx,posPy,t,time,ax,walls,wayPoints)

colors = [1 0.5 0; 0 1 0; 1 0 0;.5 0 .5; 0 0 1; 1 1 1; 1 0 1];
unColors = [.8 0.8 0.8; 1 .2 1; 0 0 0.5;.5 0 .5; 0 0 1; 1 1 1; 1 0 1];

xwall=[walls(:,1) walls(:,3)];
ywall=[walls(:,2) walls(:,4)];

plot(ax,xwall',ywall','k-');
hold(ax,'on')
robotNames = [];
rob = zeros(size(x,2)+size(posPx,2),1);
for i = 1:size(x,2)
    rob(i) = plot(ax,x(i),y(i),'o','LineWidth',1,'MarkerSize',6,...
        'MarkerEdgeColor',colors(i,:),'MarkerFaceColor',colors(i,:));
    robotNames = [robotNames, 'robot ' + string(i)];
end
for i = 1:size(posPx,2)
    rob(i+size(x,2)) = plot(ax,posPx(i),posPy(i),'*','LineWidth',1,'MarkerSize',6,...
        'MarkerEdgeColor',unColors(i,:),'MarkerFaceColor',unColors(i,:));
    robotNames = [robotNames, 'uncontrolled ' + string(i)];
end
legend(rob, robotNames,'AutoUpdate','off');
for i = 1:size(x,2)
    robAng = plot(ax,[x(i),x(i) + .25*cosd(theta(i))],[y(i),y(i) + .25*sind(theta(i))],...
    'LineWidth',3,'Color',colors(i,:));
end

% for i = 1:size(wayPoints,1)
%    plot(ax,wayPoints(i,1),wayPoints(i,2),'ko')
% end


%axis equal
ax.XLim = [-1,21];
ax.YLim = [-1,21];
%xlabel('x-pos')
%ylabel('y-pos')

grid(ax,'on')

title(ax,sprintf('time = %2.2f',time(t)))



hold(ax,'off')
end