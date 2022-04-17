clear all; clc;
close all;
addpath('..')
addpath('../util')
addpath('../set operation')
addpath('../filtering')

% Pxy Ptheta Volume versus time
%% Change Parameters
parameters      = params;
cameraTypes     = ["mono", "stereo"];

%% Derived parameter
vehicleVolume   = parameters.carLength*parameters.carWidth;
timeSteps       = getTimeSteps();
timeInterval    = parameters.updateTime/parameters.propTime;
initialDT       = timeInterval;
fontSize        = 25;
titles          = {'Monocular Camera', 'Stereo Camera'};

%% Plot isIn
fig1    = figure(1);
set(gcf,'color','w');
set(gcf,'Position',[000 000 600 400]);
for i_cam = 1:length(cameraTypes)
    subplot(2, 1, i_cam);
    camType     = cameraTypes(i_cam);
    filename    = getFileName(camType, parameters);
    data        = load(filename).Historys;
    History     = sparseCellHistory2Arr(data, initialDT);
    
    ySet        = History.SetSLAM.isIn;
    yFast       = History.FastSLAM.isIn;
    hSet1       = plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2); hold on;
    hFast1      = plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);      

    % Set axis limits and labels
    xlim([1, floor(timeSteps/initialDT)]); ylim([0,1]);
    set(gca,'FontSize', 15); grid on;
    xticks(0:20:floor(timeSteps/initialDT)); yticks([0,1]);
    xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels({'false', 'true'});
    ytickangle(90);
    title(titles{i_cam}, 'Interpreter', 'latex', 'FontSize', fontSize);
end
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ax  = axes(fig1,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'', '$\hat{P}_{xy}\in P_{xy}$ and $\hat{p}_{\theta}\in P_{\theta}$', ''}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'isInSingleRun','epsc')

%% Plot Pxy
fig2    = figure(2);
set(gcf,'color','w');
set(gcf,'Position',[000 000 600 800]);

filename    = getFileName(cameraTypes(1), parameters);
data        = load(filename).Historys;
History     = sparseCellHistory2Arr(data, initialDT);

ySet        = History.SetSLAM.P/(parameters.carLength*parameters.carWidth);
yFast       = History.FastSLAM.P/(parameters.carLength*parameters.carWidth);
subplot(4, 1, 1);
plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2);
xlim([1, floor(timeSteps/initialDT)]); ylim([0,1.5]);
set(gca,'FontSize', 15); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([0,1.5]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([0,1.5]));
ytickangle(90);
title(titles{1}, 'Interpreter', 'latex', 'FontSize', fontSize);

subplot(4, 1, 2);
plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);      
xlim([1, floor(timeSteps/initialDT)]); ylim([0,0.06]);
set(gca,'FontSize', 15); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([0,0.06]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([0,0.06]));
ytickangle(90);
title(titles{1}, 'Interpreter', 'latex', 'FontSize', fontSize);

% =========================================================================
subplot(4, 1, 3);
filename    = getFileName(cameraTypes(2), parameters);
data        = load(filename).Historys;
History     = sparseCellHistory2Arr(data, initialDT);

ySet        = History.SetSLAM.P/(parameters.carLength*parameters.carWidth);
yFast       = History.FastSLAM.P/(parameters.carLength*parameters.carWidth);
hSet1       = plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2); hold on;
hFast1      = plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);      

% Set axis limits and labels
xlim([1, floor(timeSteps/initialDT)]); ylim([0,0.03]);
set(gca,'FontSize', 15); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([0,0.03]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([0,0.03]));
ytickangle(90);
title(titles{2}, 'Interpreter', 'latex', 'FontSize', fontSize);
% =========================================================================
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
subplot(4,1,4);
set(gca,'visible','off')
legend([hSet1(1), hFast1(1)],{'Ours', 'FastSLAM'}, 'Interpreter','latex', 'NumColumns', 2, 'FontSize', fontSize*0.7, 'Position', [0.25, 0.15, 0.5,0.1]);

ax  = axes(fig2,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'', '$V(P_{i})/V(\hat{P}_{xy})$, $i=1,2,3,4$', ''}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'psSingleRun','epsc')

%% Plot Pxy
fig3    = figure(3);
set(gcf,'color','w');
set(gcf,'Position',[000 000 600 400]);
for i_cam = 1:length(cameraTypes)
    subplot(2, 1, i_cam);
    camType     = cameraTypes(i_cam);
    filename    = getFileName(camType, parameters);
    data        = load(filename).Historys;
    History     = sparseCellHistory2Arr(data, initialDT);
    
    ySet        = History.SetSLAM.PxyOr ./ History.SetSLAM.Pxy;
    yFast       = History.FastSLAM.PxyOr ./ History.FastSLAM.Pxy;
    hSet1       = plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2); hold on;
    hFast1      = plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);      

    % Set axis limits and labels
    xlim([1, floor(timeSteps/initialDT)]); ylim([0,1]);
    set(gca,'FontSize', 15); grid on;
    xticks(0:20:floor(timeSteps/initialDT)); yticks(0:0.5:1.1);
    xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string(0:0.5:1.1));
    ytickangle(90);
    title(titles{i_cam}, 'Interpreter', 'latex', 'FontSize', fontSize);
end
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ax  = axes(fig3,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'', '$V(P_{xy}\cap\hat{P}_{xy})/V(P_{xy})$', ''}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'pxySingleRun','epsc')

%% Plot Pt
fig4    = figure(4);
set(gcf,'color','w');
set(gcf,'Position',[000 000 600 800]);
% =========================================================================
subplot(4, 1, 1);
filename            = getFileName(cameraTypes(1), parameters);
History             = sparseCellHistory2Arr(load(filename).Historys, initialDT);
patchX              = [1:floor(timeSteps/initialDT), flip(1:floor(timeSteps/initialDT))];
History.pt(70:end)  = History.pt(70:end) + 2*pi;

for i = 65:floor(timeSteps/initialDT)
    if History.SetSLAM.PtInf(i) < 0 || History.SetSLAM.PtSup(i) < 0
        History.SetSLAM.PtInf(i)    = History.SetSLAM.PtInf(i)+2*pi;
        History.SetSLAM.PtSup(i)    = History.SetSLAM.PtSup(i)+2*pi;
    end

    if History.FastSLAM.PtInf(i) < 0 || History.FastSLAM.PtSup(i) < 0
        History.FastSLAM.PtInf(i)   = History.FastSLAM.PtInf(i)+2*pi;
        History.FastSLAM.PtSup(i)   = History.FastSLAM.PtSup(i)+2*pi;
    end
end
History.FastSLAM.PtInf(122:124) = History.FastSLAM.PtInf(122:124)+2*pi;
History.FastSLAM.PtSup(122:124) = History.FastSLAM.PtSup(122:124)+2*pi; 
History.FastSLAM.PtInf(20)      = History.FastSLAM.PtInf(20)-2*pi;
History.FastSLAM.PtSup(20)      = History.FastSLAM.PtSup(20)-2*pi;

setPy   = [History.SetSLAM.PtInf-History.pt, flip(History.SetSLAM.PtSup-History.pt)];       
FastPy  = [History.FastSLAM.PtInf-History.pt, flip(History.FastSLAM.PtSup-History.pt)];
patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25); hold on;
plot(1:floor(timeSteps/initialDT), zeros(size(History.pt)), 'r', 'LineWidth', 1);

xlim([1, floor(timeSteps/initialDT)]); ylim([deg2rad(-90), deg2rad(90)]);
set(gca,'FontSize', 15); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([deg2rad(-90), deg2rad(90)]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([-90, 90]));
ytickangle(90);
title(titles{1}, 'Interpreter', 'latex', 'FontSize', fontSize);
% =========================================================================
subplot(4, 1, 2);
patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25); hold on;
plot(1:floor(timeSteps/initialDT), zeros(size(History.pt)), 'r', 'LineWidth', 1);

xlim([1, floor(timeSteps/initialDT)]); ylim([deg2rad(-15), deg2rad(27)]);
set(gca,'FontSize', 15); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([deg2rad(-15), deg2rad(27)]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([-15, 27]));
ytickangle(90);
title(titles{1}, 'Interpreter', 'latex', 'FontSize', fontSize);
% =========================================================================
subplot(4, 1, 3);
filename            = getFileName(cameraTypes(2), parameters);
History             = sparseCellHistory2Arr(load(filename).Historys, initialDT);
patchX              = [1:floor(timeSteps/initialDT), flip(1:floor(timeSteps/initialDT))];
History.pt(70:end)  = History.pt(70:end) + 2*pi;

for i = 63:floor(timeSteps/initialDT)
    if History.SetSLAM.PtInf(i) < 0 || History.SetSLAM.PtSup(i) < 0
        History.SetSLAM.PtInf(i)    = History.SetSLAM.PtInf(i)+2*pi;
        History.SetSLAM.PtSup(i)    = History.SetSLAM.PtSup(i)+2*pi;
    end

    if History.FastSLAM.PtInf(i) < 0 || History.FastSLAM.PtSup(i) < 0
        History.FastSLAM.PtInf(i)   = History.FastSLAM.PtInf(i)+2*pi;
        History.FastSLAM.PtSup(i)   = History.FastSLAM.PtSup(i)+2*pi;
    end
end

setPy   = [History.SetSLAM.PtInf-History.pt, flip(History.SetSLAM.PtSup-History.pt)];       
FastPy  = [History.FastSLAM.PtInf-History.pt, flip(History.FastSLAM.PtSup-History.pt)];
hSet1   = patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25); hold on;
hFast1  = patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25);
hpt     = plot(1:floor(timeSteps/initialDT), zeros(size(History.pt)), 'r', 'LineWidth', 1);

xlim([1, floor(timeSteps/initialDT)]); ylim([deg2rad(-34), deg2rad(8)]);
set(gca,'FontSize', 15); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([deg2rad(-34), deg2rad(8)]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([-34, 8]));
ytickangle(90);
title(titles{2}, 'Interpreter', 'latex', 'FontSize', fontSize);
% =========================================================================
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
subplot(4,1,4);
set(gca,'visible','off')
legend([hSet1, hFast1, hpt],{'[min$P_{\theta}$-$\hat{p}_{\theta}$, max$P_{\theta}$-$\hat{p}_{\theta}$] (Ours)',...
    '[min$P_{\theta}$-$\hat{p}_{\theta}$, max$P_{\theta}$-$\hat{p}_{\theta}$] (FastSLAM)', 'zero'}, 'Interpreter','latex', 'NumColumns', 2, 'FontSize', fontSize*0.7, 'Position', [0.25, 0.15, 0.5,0.1]);

ax  = axes(fig4,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'', '(max/min) $P_{\theta}$ - $\hat{p}_{\theta}$ [deg]', ''}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'ptSingleRun','epsc')

%% Support functions
function filename = getFileName(cameraType, pr)
    filename    = "snapshot_" + cameraType + "_eva_" + num2str(pr.e_va) + "_evr_" + num2str(pr.e_vr) + ...
                    "_eSteering_" + num2str(pr.e_steering) + "_eVelocity_" + num2str(pr.e_velocity) + ...
                    "_Lt0_" + num2str(pr.epsilon_Lt) + "_Lxy0_" + num2str(pr.epsilon_Lxy) + "_P0_" + num2str(pr.epsilon_P) + '.mat';
end

function History = sparseCellHistory2Arr(data, initialDT)
    % remove the initialization timestep
    History.pt              = zeros(1, floor(length(data)/initialDT));
    
    History.SetSLAM.Pxy     = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.PtInf   = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.PtSup   = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.P       = zeros(4, floor(length(data)/initialDT));
    History.SetSLAM.isIn    = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.PxyOr   = zeros(1, floor(length(data)/initialDT));
    
    History.FastSLAM.Pxy    = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.PtInf  = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.PtSup  = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.P      = zeros(4, floor(length(data)/initialDT));
    History.FastSLAM.isIn   = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.PxyOr  = zeros(1, floor(length(data)/initialDT));
    for i = 1:floor(length(data)/initialDT) % remove the initialization timestep
        History.pt(i)               = data{i*initialDT+1}.theta_car;
        
        History.SetSLAM.Pxy(i)      = data{i*initialDT+1}.SetSLAM.Pxy;
        History.SetSLAM.PtInf(i)    = data{i*initialDT+1}.SetSLAM.PtInf;
        History.SetSLAM.PtSup(i)    = data{i*initialDT+1}.SetSLAM.PtSup;
        History.SetSLAM.P(:,i)      = data{i*initialDT+1}.SetSLAM.P;
        History.SetSLAM.isIn(i)     = data{i*initialDT+1}.SetSLAM.isIn;
        History.SetSLAM.PxyOr(i)    = data{i*initialDT+1}.SetSLAM.PxyOr;
        
        History.FastSLAM.Pxy(i)     = data{i*initialDT+1}.FastSLAM.Pxy;
        History.FastSLAM.PtInf(i)   = data{i*initialDT+1}.FastSLAM.PtInf;
        History.FastSLAM.PtSup(i)   = data{i*initialDT+1}.FastSLAM.PtSup;
        History.FastSLAM.P(:,i)     = data{i*initialDT+1}.FastSLAM.P;
        History.FastSLAM.isIn(i)    = data{i*initialDT+1}.FastSLAM.isIn;
        History.FastSLAM.PxyOr(i)   = data{i*initialDT+1}.FastSLAM.PxyOr;
    end
end

function timeSteps = getTimeSteps()
    data    = load('../Path.mat').Historys;
    timeSteps   = length(data);
end