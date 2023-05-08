clear
clc
close all


% Settings 1 on 0 off. Only 1 at a time
findWaypoints = 1;
findWalls = 0;    
addoffset = 0;

% Filenames
fileOfI = 'hallwaymap';

yamlFile = readlines(strcat(fileOfI,'.yaml'));
offString = yamlFile(3);
offSplit = regexp(offString,'\d+\.?\d*','match');
offS = -[str2double(offSplit(1)),str2double(offSplit(2))];

image = imread(strcat(fileOfI,'.pgm'));
imshow(image)

imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;


occMap = occupancyMap(imageOccupancy,50);
occMap.GridOriginInLocal = offS;
show(occMap)
hold on

try
    oldWalls = cell2mat(readcell(strcat(fileOfI, '_walls.txt')));

    if addoffset == 1
        oldWalls(:,1) = oldWalls(:,1) + offS(1);
        oldWalls(:,3) = oldWalls(:,3) + offS(1);
        oldWalls(:,2) = oldWalls(:,2) + offS(2);
        oldWalls(:,4) = oldWalls(:,4) + offS(2);
    end
    for i = 1:size(oldWalls,1)
        plot([oldWalls(i,1),oldWalls(i,3)],[oldWalls(i,2),oldWalls(i,4)],'b-','lineWidth',3)
    end
catch
end

try
    oldWaypoints = cell2mat(readcell(strcat(fileOfI,'_waypoints.txt')));

    if addoffset == 1
        oldWaypoints = oldWaypoints + offS;
    end
    for i = 1:size(oldWaypoints,1)
        plot(oldWaypoints(i,1),oldWaypoints(i,2), 'ro')
    end
catch
end

plot(33, -18.25, 'bo')
% xlim([-10 20]);
% ylim([-10 10]);
% n is the number of points you want to click
n = 0;
[x,y] = ginput(n);

if findWalls == 1
    point1x = x(1:2:end);
    point2x = x(2:2:end);
    point1y = y(1:2:end);
    point2y = y(2:2:end);
    
    walls = [point1x, point1y, point2x, point2y];
    
    try
        newWalls = [walls;oldWalls];
    catch
        newWalls = walls;
    end
    wallFile = strcat(fileOfI,'_walls.txt');
    writematrix(newWalls,wallFile,'Delimiter','space')
end

if findWaypoints == 1
    try
        waypoints = [x,y];
        newWaypoints = [waypoints;oldWaypoints];
    catch
        newWaypoints = waypoints;
    end
    
    waypointsFile = strcat(fileOfI,'_waypoints.txt');
    writematrix(newWaypoints,waypointsFile,'Delimiter','space')

end
hold on
plot(20.88,66.32,'ro')
offset = [20.88, 66.32];

point = [3.5 -3.7];
plot(offset(1)+point(1), offset(2)+point(2),'bo')






