clear
clc
close all

load('paperdata1.mat')
figure('Name','STL')
%writerObj = VideoWriter('STL6.avi');
%open(writerObj);

%timeSteps = linspace(1,400,400);%[1,35,140,210,300,400];
walls = [0, 0, 0, 20;0, 20, 20, 20; 20, 20, 20, 0;20, 0, 0, 0;6.66, 0, 6.66, 1;
    6.66, 4, 6.66, 9; 6.66, 12, 6.66, 15;13.3, 0, 13.3, 3; 13.3, 6, 13.3, 7.5; 
    13.3, 10.5, 13.3, 15;0, 6.66, 6.66, 6.66;0, 15, 7.5, 15;13.3, 6.66, 20, 6.66;
    12.5, 15, 20, 15];
xwall=[walls(:,1) walls(:,3)];
ywall=[walls(:,2) walls(:,4)];
timeSteps = [1,25,40,60,80,100,120,150];
timeSteps = linspace(1,63,63);
for i = 1:length(timeSteps)

    toTime = timeSteps(i);
    plot(x(1,1:toTime),y(1,1:toTime),'k-')
    %plot(x(1,1:toTime),y(1,1:toTime),'b--','linewidth',4)
    ax = gca
    ax.LineWidth = 2
     xlim([-1 21])
     xticks([0 5 10 15 20])
     yticks([0 5 10 15 20])
     ylim([-1 21])
%     xlim([-2,6])
%     xticks([-2 0 2 4 6])
%     yticks([-2 0 2 4 6])
%     ylim([-2,6])
    %xlabel('x-position','FontSize', 28)
    %ylabel('y-position','FontSize', 28)
    %grid on
    
    hold on
    plot(ax,xwall',ywall','k-');
    plot(ax,x(1,1:toTime),y(1,1:toTime),'rs','markersize',10,'markerfacecolor','r')
    plot(ax,x(2,1:toTime),y(2,1:toTime),'bd','markersize',10,'markerfacecolor','b')

    %plot(x(3,1:toTime),y(3,1:toTime),'k-')
    %plot(x(4,1:toTime),y(4,1:toTime),'k-')
    plot(x(1,toTime),y(1,toTime),'k.','markersize',35)
    %plot([x(1,toTime),x(1,toTime)+.25*cos(theta(1,toTime))],[y(1,toTime),y(1,toTime)+.25*sin(theta(1,toTime))],'k-','linewidth',3)
    plot(x(2,toTime),y(2,toTime),'ko','markersize',10)
    %plot([x(2,toTime),x(2,toTime)+.25*cos(theta(2,toTime))],[y(2,toTime),y(2,toTime)+.25*sin(theta(2,toTime))],'k-','linewidth',3)
    %plot(x(3,toTime),y(3,toTime),'ks','markersize',10)
    %plot([x(3,toTime),x(3,toTime)+.25*cos(theta(3,toTime))],[y(3,toTime),y(3,toTime)+.25*sin(theta(3,toTime))],'k-','linewidth',3)
    %plot(x(4,toTime),y(4,toTime),'kd','markersize',10)
    %plot([x(4,toTime),x(4,toTime)+.25*cos(theta(4,toTime))],[y(4,toTime),y(4,toTime)+.25*sin(theta(4,toTime))],'k-','linewidth',3)

   
    set(gca,'DataAspectRatio',[1,1,1])
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel',a,'fontsize',28)
    fname = sprintf('simfig%s.png',string(i));
    %fname = sprintf('barrier%s.png',string(i));
    %saveas(gcf,fname)
    title(sprintf('time = %2.2f',time(i)))
    %frame = getframe(gcf);
    %writeVideo(writerObj, frame);
    pause(.1)
    hold off
    
end
 %close(writerObj)

