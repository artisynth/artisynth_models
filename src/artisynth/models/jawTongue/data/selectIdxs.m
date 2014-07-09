function output_txt = selectIdxs(obj,event_obj)
% Display the position of the data cursor
% obj          Currently not used (empty)
% event_obj    Handle to event object
% output_txt   Data cursor text string (string or cell array of strings).

%
% -- script finds a matching point in pts.mat and adds its idx to idx.mat
%
% 1) enable callback with datacursormode() call
% 2) clear idxs list by 
%    >> idxs = [];
%    >> save 'idxs.mat' idxs
%
% Ian Stavness -- 28/01/2010


pos = get(event_obj,'Position');
output_txt = {['X: ',num2str(pos(1),4)],...
    ['Y: ',num2str(pos(2),4)]};

% If there is a Z-coordinate in the position, display it as well
if length(pos) > 2
    output_txt{end+1} = ['Z: ',num2str(pos(3),4)];
end

load idxs
load pts
if (size(pos,2) ~= size(pts,2))
    disp('wrong pts size');
    return;
end
idx = find(sum(abs(pts-repmat(pos,length(pts),1)),2) < 1e-6);

if (size(idx) == [1,1])
        disp(sprintf('found new idx %f',idx));
        idxs = [idxs; idx]
else
    disp('did not get one idx');
    idx
end
save 'idxs.mat' idxs