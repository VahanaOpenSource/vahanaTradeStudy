% Function to name and remember the names of figures

function h = figuren(name)

% Get all figure handles
figHandles = findobj('Type','figure');

found = false;
for i = 1:length(figHandles)
    if strcmp(figHandles(i).Name,name)
        % Already exists, make active
        figure(figHandles(i));
        h = figHandles(i);
        found = true;
        break;
    end
end

if ~found
    h = figure('name',name,'numbertitle','off');
end