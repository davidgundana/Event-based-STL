function updatePerson(animate,ax,posPx,posPy,maxP,tStep)
    a = '';
    keyPress = set(animate,'KeyPressFcn',@(keyPress,event) assignin('base','a',event.Key));
    disp(a)
    a = string(a);
    if strcmp(a,'uparrow')
        cmdPx = 0;
        cmdPy = 1;
    elseif strcmp(a,'downarrow')
        cmdPx = 0;
        cmdPy = -1;
    elseif strcmp(a,'rightarrow')
        cmdPx = 1;
        cmdPy = 0;
    elseif strcmp(a,'leftarrow')
        cmdPx = -1;
        cmdPy = 0;
    else
        cmdPx = 0;
        cmdPy = 0; 
    end
    a = '';
    pause(.05)
    
    posPx = posPx + (cmdPx*maxP)*(1/tStep);
    posPy = posPy + (cmdPy*maxP)*(1/tStep);
    %hold(ax,'on')
    plot(ax,posPx,posPy,'bo')
    hold(ax,'off')
end

