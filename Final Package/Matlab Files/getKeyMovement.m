function [cmdPx,cmdPy] = getKeyMovement(animate)
    % Get keyboard input to move the human
    p = get(animate, 'CurrentCharacter');
    p = string(p);
    cmdPx = 0;
    cmdPy = 0;
    switch p
        case 'w'
            cmdPx = 0;
            cmdPy = 1;
        case 's'
            cmdPx = 0;
            cmdPy = -1;
        case 'd'
            cmdPx = 1;
            cmdPy = 0;
        case 'a'
            cmdPx = -1;
            cmdPy = 0;
    end
    %If keyboard is not responsive it may be necessary to increase this
    %pause
    pause(.00001)
end

