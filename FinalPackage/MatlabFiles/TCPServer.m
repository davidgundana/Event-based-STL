function TCPServer(k_max,time,maxP,tStep,robots ,n_people,init_robot,init_human,input,map,inputNames,nodes)

t = tcpip('0.0.0.0', 8052, 'NetworkRole', 'server');
t.InputBufferSize = 500;
t.OutputBufferSize = 1024;
t.Terminator = '';
t.timeout = .00001;

%Start time of execution
startTime = 0;
%State of execution(starts at 0)
currState = 0;
%current time
currTime = 0;
%Run has not started yet
started = 0;
%size of header to signify message complete
headersize = 75;
%Toggle for plotting and calculating time derivatives of barrier functions
bDeriv = 0;
%Toggle for calculating derivatie for distance
distTog = 0;
screenshot = 0; 
maxV = 1;
inp = length(input)/2;
newinput = input;
%Setup initial position for consistent plotting
for i = 1:robots
    posX(i) = init_robot(3*(i-1)+1);
    posY(i) = init_robot(3*(i-1)+2);
    posTheta(i) = init_robot(3*(i-1)+3);
    
    initX(i) = init_robot(3*(i-1)+1); 
    initY(i) = init_robot(3*(i-1)+2); 
    initTheta(i) = init_robot(3*(i-1)+3);
end
numPeople = size(init_human,2)/3;
posPx = 0; posPy = 0; posPTheta =0;
for i = 1:numPeople
    posPx(i) = init_human(3*(i-1)+1);
    posPy(i) = init_human(3*(i-1)+2);
    posPTheta(i) = init_human(3*(i-1)+3);
end

%Setup plotting
[ax,start,stop,btn,animate] = plotLiveSetup(inp,inputNames);
%Plot the first points
[rob,robAng] = plotLive(posX,posY,posTheta,posPx,posPy,1,time,ax,map,nodes);

%TEMPORARY Uncomment to make circles signifying safe sets for screenshots
% if screenshot == 1
%     hold(ax,'on')
%     plot(ax,7.5,2,'o','LineWidth',1,'MarkerSize',8,...
%         'MarkerEdgeColor',[1 0.5 0],'MarkerFaceColor',[1 0.5 0])
%     circle(ax,[2.5,12.5],2.5,1000,'b-')
%     circle(ax,[5,14],1,1000,'b-')
% end

%Wait for the start button to begin the simulation
while started < 1
    started = get(start,'userdata');
    pause(.3)
end

%Open the TCP connection and send a quick message to establish
%communication
fopen(t);
firstMess = num2str(robots) + " " + num2str(size(input,2)/2) + " " + num2str(n_people);
fwrite(t, firstMess, 'char');

%Loop for length of simulation
for k = 1:k_max
    tic
    %Log all info
    time(k) = k*(1/tStep);
    for i = 1:robots
        pos_robotX(i,k) = posX(i);
        pos_robotY(i,k) = posY(i);
        pos_robotTheta(i,k) = posTheta(i);
    end
    for i = 1:n_people
        pos_humanX(i,k) = posPx(i);
        pos_humanY(i,k) = posPy(i);
        pos_humanTheta(i,k) = posPTheta(i);
%         pos_humanX(i,k) = posX(i);
%         pos_humanY(i,k) = posY(i);
%         pos_humanTheta(i,k) = posTheta(i);
    end
        
    currTime = k/tStep - 1/tStep;
    
    pause(.01)
    for i = 1:inp
        input(2*i-1) = get(btn(i),'userdata');
        if get(btn(i),'userdata') ==1 && input(2*i) == 0
            input(2*i) = k/tStep;
        end
    end
%     if k == 20
%        input(1) = 1;
%        input(2) = 2;
%     end
%     if k == 100
%         input(3) = 1;
%         input(4) = 10;
%     end
%     if k == 120
%         input(1) = 1;
%         input(2) = 12;
%     end
%     if k == 180
%         input(5) =1;
%         input(6) = 18;
%     end
    
    if inp == 0
        input = 0;
    end
    %create message for python script
    %Current format of the message is
    %[posX,posY,posTheta,initX,initY,initTheta,posPx,posPy,posPtheta,currtime,
    %startTime,currState,input1,input2];
    message = "";
    for i = 1:robots
       message = message + num2str(pos_robotX(i,k)) + " " + num2str(pos_robotY(i,k)) + " " + ...
           num2str(pos_robotTheta(i,k)) + " ";    
    end
    for i = 1:robots
       message = message + num2str(initX(i)) + " " + num2str(initY(i)) + " " + ...
           num2str(initTheta(i)) + " ";    
    end
    for i = 1:n_people
       message = message +  num2str(pos_humanX(i,k)) + " " + num2str(pos_humanY(i,k)) + " " + ...
           num2str(pos_humanTheta(i,k)) + " ";
    end
    message = message + num2str(currTime) + " " + num2str(startTime) + " " + num2str(currState) + " ";
    for i = 1:size(input,2)
        if i < size(input,2)
            message = message + num2str(input(i)) + " ";
        else
            message = message + num2str(input(i));
        end
    end
 
    msglength = int2str(strlength(message));
    msg = pad(msglength,headersize) + message;
    
    %send message to python
    fwrite(t,msg,'char')
    disp('DEBUG MESSAGE: ' + message)
    %Get keyboard movement for human and change the position of the human
    [cmdPx,cmdPy] = getKeyMovement(animate);
    posPx = posPx + (cmdPx*maxP)*(1/tStep);
    posPy = posPy + (cmdPy*maxP)*(1/tStep);
    %This has to be set so that the robot stops moving between keystrokes
    set(animate,'CurrentCharacter','z')
    
    %Grab the data from python stream
    data = fscanf(t);
    %Retry again if neccessary. Sometimes timing is off
    if isempty(data)
         disp('Could not get data. Retrying')
         fwrite(t,msg,'char')
         data = fscanf(t);
         if isempty(data)
             data = olddata;
             save('paperdata1')
             break
             
%             disp('Could not get data. Retrying agai')
%             fwrite(t,msg,'char')
%             data = fscanf(t);        
         end
    end
    
    olddata = data;
    %Parse Data that is recieved
    splitMessage = strsplit(data,' ');
    for i = 1:robots
        cmdVx(i) = str2num(splitMessage{3*(i-1)+1});
        cmdVy(i) = str2num(splitMessage{3*(i-1)+2});
        cmdVtheta(i) = str2num(splitMessage{3*(i-1)+3});
    end
    currState = str2num(splitMessage{robots*3+1});
    
    sizeOfInput = size(splitMessage,2) - 3*robots - 2;
    for i = 1:sizeOfInput
        newinput(i) = str2num(splitMessage{3*robots+1+i});
    end
    
    for i = 1:inp
        if input(2*i-1) ~=  newinput(2*i-1)
           btn(i).UserData = 0;
           input(2*i-1:2*i) =  newinput(2*i-1:2*i);
        end 
    end
    input = newinput;
    %update all positions and plot them
    for i = 1:robots
        posX(i) = posX(i) + cmdVx(i) *(1/tStep);
        posY(i) = posY(i) + cmdVy(i) *(1/tStep);
        posTheta(i) = posTheta(i) + cmdVtheta(i) *(1/tStep);
    end
    
    delete(rob)
    delete(robAng)
    [rob,robAng] = plotLive(posX,posY,posTheta,posPx,posPy,k,time,ax,map,nodes);
    
    
    pause(.01)
    stopped = get(stop,'userdata');
    if stopped == 1
        break
        fclose(t);
    end
    %toc
    
    x(:,k) = posX';
    y(:,k) = posY';
    theta(:,k) = posTheta';
end




save('paperdata1')
fclose(t);
