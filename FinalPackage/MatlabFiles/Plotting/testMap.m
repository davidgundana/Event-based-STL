clear
clc
close all

load('exampleMap.txt')
load('exampleNodes.txt')
xwall=[exampleMap(:,1) exampleMap(:,3)];
ywall=[exampleMap(:,2) exampleMap(:,4)];
plot(exampleNodes(:,1),exampleNodes(:,2),'bo')
rectangle('Position',[-5,30,50,10],'FaceColor',[0 0 0])
rectangle('Position',[-5,-10,50,10],'FaceColor',[0 0 0])
rectangle('Position',[-5,-5,5,40],'FaceColor',[0 0 0])
rectangle('Position',[40,-5,5,50],'FaceColor',[0 0 0])
rectangle('Position',[8,-5,24,7],'FaceColor',[0 0 0])
rectangle('Position',[11,6,4,3],'FaceColor',[0 0 0])
hold on 
plot(xwall',ywall','r-','linewidth',1.2);
hold off
axis equal
xlim([-2, 42])
ylim([-2, 32])
