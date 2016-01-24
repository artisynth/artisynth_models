

close all;
files = {'ex_GGA_output.txt','ex_GGM_output.txt','ex_HG_output.txt','ex_VERT_output.txt'};

act = [];
for s=files
    pd = asread(cell2mat(s));
    act = [act, pd.data];
    time = pd.time;
end

idx = 401:1101;

% plot(time(idx)-time(idx(1)),data(idx,:))
labels = {'ContactArea','GGA', 'GGM', 'HG', 'VERT'};

areadata = asread('area.txt');
area = areadata.data;

act(:,3) = act(:,3)*1.005; % so visible
act(:,1) = act(:,1)*9.99; % so visible

fh = figure;
figsize = [0,1000,800,400];
fh.Position = figsize;

t = time(idx)-time(idx(1));
[h,p1,p2] = plotyy(t,area(idx), t,100*data(idx,:));

set(h,{'ycolor'},{'k';'k'})
set(h,{'FontSize'},{18;18})

xlabel(h(1),'Time (s)'); % label x-axis
ylabel(h(2),'Muscle Activation'); % label left y-axis
ylabel(h(1),'Contact Area (mm^2)'); % label right y-axis

maxcontact = 380;
h(1).YLim = [0 maxcontact];
% h(2).YLim = [0 maxcontact/400*25];
h(2).YTick = [0,5,10,15];
h(2).YTickLabels={'0%','5%','10%','15%'};
% h(2).YTick = [0,5,10,15,20];
% h(2).YTickLabels={'0%','5%','10%','15%','20%'};

for i=1:numel(p2)
    p2(i).LineWidth = 2;
end

p1.LineWidth = 2;
p1.LineStyle = '--';
p1.Color = 'k';

legend(labels)
% 
% set(gcf, 'PaperPositionMode', 'auto') % So that saved image is the size of the screen
% print('contactarea','-dpng')
% 
% fh = gcf;
% fh.PaperUnits = 'points';
% fh.PaperPosition = figsize;
% D = fh.PaperPosition;
% fh.PaperSize = [D(3) D(4)];
% fh.PaperPositionMode = 'auto';
% saveas(fh,['contactarea.eps'],'psc2');
