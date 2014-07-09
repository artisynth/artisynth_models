%
%  plotjt.m
%  -- script to visualize data from jaw and jaw-tongue simulations
%
% Ian Stavness -- 14/Jan/2010

 
close all;

%% gather data
models = {'jaw', 'jawtongue'}';
tasks = {'clench', 'rest','open','hingeopen','protrude','rlat','rchew'}';
landmarks = {'pos_lowerincisor.txt'}';

step = 0.01;
interval = [0, 0.6];
N = (interval(2)-interval(1))/step; % number of time samples
data = zeros(length(models), length(tasks), length(landmarks), N, 3);
li = 1; % incisor point

for mi = 1:length(models)
    for ti = 1:length(tasks)
        [pdata time] = asload( ...
            ['jtdata/',models{mi},'/',tasks{ti},'/',landmarks{li}], ...
            interval(1), interval(2));
        if (length(time)~=N)
            disp(sprintf('bad data size for %s,%s',models{mi},tasks{ti}));
            return;
        end
        displacement = pdata-repmat(pdata(1,:),N,1);
        data(mi,ti,li,:,:) = displacement;
    end
    
end

set(0,'defaultAxesFontSize',14)

%% plot all models for each task
% for ti = 1:length(tasks)
%     figure;
%     for mi = 1:length(models)
%         pdata = squeeze(data(mi,ti,li,:,:));
%         scatter3(pdata(:,1),pdata(:,2),pdata(:,3))
%         hold on;
%         title(upper(tasks{ti}));
%         xlabel('anterior-posterior')
%         ylabel('left-right')
%         zlabel('inferior-superior')
%         axis equal
%     end
%     legend(models);
% end


% set(gca, 'NextPlot', 'replacechildren')

%% plot all tasks for each model

% for mi = 1:length(models)
%     figure;
%     allpdata = squeeze(data(mi,:,li,:,:));
%     plot3(allpdata(:,:,1)', allpdata(:,:,2)', allpdata(:,:,3)');
%     axis equal;
% end

for mi = 1:length(models)
    figure('Name',models{mi});
    allpdata = squeeze(data(mi,:,li,:,:));
    plot3(allpdata(:,:,1)', allpdata(:,:,2)', allpdata(:,:,3)');
    hold on;
%     set(gca, 'NextPlot', 'replacechildren')
    for ti = 1:length(tasks)
        pdata = squeeze(data(mi,ti,li,:,:));
        scatter3(pdata(:,1),pdata(:,2),pdata(:,3),'.');
        hold on;
%         title(upper(models{mi}));
        xlabel('anterior-posterior')
        ylabel('right-left')
        zlabel('inferior-superior')
        axis equal
    end
    legend(tasks);
    set(gca, 'YDir', 'reverse');
    view([0,0]); % sagittal
    print('-depsc',['plots/',models{mi},'_sagittal']);
    view([90,0]); % sagittal
    print('-depsc',['plots/',models{mi},'_frontal']);
end



%% displacement versus time plot
% figure;
% t = (1:N)'*step;
% mi = 1;
% ti = 3;
% pdata = squeeze(data(mi,ti,li,:,:));
% plot(t,disp);
% xlabel('time (s)');
% ylabel('displacement (mm)');
% title([upper(models{mi}),', ',upper(tasks{ti})]);
% legend('-right +left','-anterior +posterior','-inferior +superior');


