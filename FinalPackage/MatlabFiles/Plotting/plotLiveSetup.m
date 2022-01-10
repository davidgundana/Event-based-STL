function [ax,start,stop,btn,animate] = plotLiveSetup(inp,inputNames)
animate = uifigure;
animate.Position = [400 400 650 420];
ax = uiaxes('Parent',animate,...
    'Units','pixels',...
    'Position', [10,10, 400, 400]);
start = uibutton(animate,'push','Text','Start',...
            'Position',[425,375, 100, 22],...
            'ButtonPushedFcn', @(start,event) plotStart(start,ax));
set(start,'userdata',0')
stop = uibutton(animate,'push','Text','Stop',...
            'Position',[425,350, 100, 22],...
            'ButtonPushedFcn', @(stop,event) plotStop(stop,ax));
set(stop,'userdata',0')
count = 1;
for i = 1:inp
    message = string(inputNames(i,:)) +'  -  True';
%     lmp(count) = uilamp(animate,...
%     'Position',[165 75 20 20],...
%     'Color','green');
    btn(count) = uiswitch(animate,'slider','Items',{char(message),'False'},...
        'Position',[525, 180-25*(i-1), 100, 22],...
        'ValueChangedFcn', @(btn,event) plotButtonPushed(btn,count,ax));
    btn(count).Value = 'False';
    set(btn(count),'userdata',0')

    count = count + 1;
end
if inp == 0
    btn = 0;
end

function plotButtonPushed(btn,count,ax)
   if strcmp(btn.Value,'False')
       set(btn,'userdata',0)
   else
       set(btn,'userdata',1)
   end
%    if strcmp(btn.Value,'False')
%         lmp(count).Color = 'green';
%    else
%         lmp(count).Color = 'red';
%     end
end

end

