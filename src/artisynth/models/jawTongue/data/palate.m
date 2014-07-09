%
% palate.m
% -- script to visualize mid-sagittal contours of jaw and tongue meshes
%
%
% Ian Stavness - 25/01/2010


close all;

%% gather data
datadir = 'jtdata/jawtongue/';
models = {'jaw','hyoid','maxilla','tongue'}';

% tasks = {'palatefast'}';
% interval = [0.0, 0.8];
% timeIdxsToPlot = [40:5:65];

tasks = {'palateslow'}';
interval = [0.0, 1.1];
timeIdxsToPlot = [40,60,65,70,90,95];

axisdims = {[-5 75 -40 30]};

step = 0.01;

N = (interval(2)-interval(1))/step; % number of time samples

initinc = [56.703720798903994 93.702185360807];

data = cell(length(models),length(tasks),1);
idxs = cell(length(models),1);

for mi = 1:length(models)
    for ti = 1:length(tasks)
        filename = [datadir,tasks{ti},'/',models{mi},'Contour.txt'];
        [pdata time] = asload(filename, ...
                interval(1), interval(2));
        if (length(time)~=N)
            disp(sprintf('bad data size for %s',models{mi}));
            return;
        end
        if (strcmp(models{mi},'tongue'))
           % tongue contour has 3D data
            data3d = reshape(pdata,N,3,size(pdata,2)/3);
            data2d = data3d(:,[1,3],:);
        else
            data2d = reshape(pdata,N,2,size(pdata,2)/2);
        end
        data{mi, ti} = data2d-repmat(initinc, [N, 1, size(data2d,3)]);
        contouridxs = load([models{mi},'ContourIdxs.mat']);
        idxs{mi} = contouridxs.idxs;
    end
end


colors = ['r';'m';'b';'k'];
set(0,'defaultAxesFontSize',14)


%% plot all models for each time step
labels = {};
% figure;
for ti = 1:length(tasks)
for i = timeIdxsToPlot
    timestr = num2str(1000*(time(i)-time(timeIdxsToPlot(1))));
    figure('Name',['jawtongue ', tasks{ti},', t=',timestr]);
    for mi = 1:length(models)
        pdata = squeeze(data{mi, ti}(i,:,:))';
%         k = [1:size(pdata,1),1];
%         k = findloop(pdata(:,1),pdata(:,2));
        k = idxs{mi};
%         scatter(pdata(:,1),pdata(:,2))
        plot(pdata(k,1),pdata(k,2), colors(mi));
        hold on;
        scatter(pdata(k,1),pdata(k,2), ['.',colors(mi)]);
        hold on;
        axis equal
%         labels = [labels, [models{mi},', t=',num2str(time(i))]];
    end
    title(['time = ', timestr, ' ms']);
    xlabel('anterior-posterior')
    ylabel('inferior-superior')
    print('-depsc',['plots/',tasks{ti},'_contour_',timestr]);
end
end
% legend(labels)