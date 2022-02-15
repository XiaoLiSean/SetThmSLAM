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
timeSpan        = parameters.simLoopDt * timeSteps;
initialDT       = 100;
fontSize        = 20;
titles          = {'Monocular Camera', 'Stereo Camera'};

%% Plot isIn
fig     = figure(1);
set(gcf,'color','w');
set(gcf,'Position',[000 000 2000 600])
for i_cam = 1:length(cameraTypes)
    subplot(3, 3, i_cam*3 - 2);
    camType     = cameraTypes(i_cam);
    filename    = getFileName(camType, parameters);
    data        = load(filename).Historys;
    History     = cellHistory2Arr(data, initialDT);
    
    ySet        = History.SetSLAM.isIn;
    yFast       = History.FastSLAM.isIn;
    hSet1       = plot(1:(timeSteps-initialDT), ySet, 'r', 'LineWidth', 2); hold on;
    hFast1      = plot(1:(timeSteps-initialDT), yFast, 'b', 'LineWidth', 2);      

    % Set axis limits and labels
    xlim([1, timeSteps-initialDT]); ylim([min([ySet, yFast], [], 'all'), max([ySet, yFast], [], 'all')]);
    set(gca,'FontSize', 15); grid on;
    xticks(0:1e3:7e3); yticks([0,1]);
    xticklabels(string(0:1e3:7e3)); yticklabels({'false', 'true'});
    title(titles{i_cam}, 'Interpreter', 'latex', 'FontSize', fontSize);
end
ylabel({'$\hat{P}_{xy}\in P_{xy}$ and $\hat{p}_{\theta}\in P_{\theta}$', ''}, 'Interpreter','latex', 'FontSize', fontSize);

%% Plot Pxy
for i_cam = 1:length(cameraTypes)
    subplot(3, 3, i_cam*3 - 1);
    camType     = cameraTypes(i_cam);
    filename    = getFileName(camType, parameters);
    data        = load(filename).Historys;
    History     = cellHistory2Arr(data, initialDT);
    
    ySet        = History.SetSLAM.PxyOr ./ History.SetSLAM.Pxy;
    yFast       = History.FastSLAM.PxyOr ./ History.FastSLAM.Pxy;
    hSet1       = plot(1:(timeSteps-initialDT), ySet, 'r', 'LineWidth', 2); hold on;
    hFast1      = plot(1:(timeSteps-initialDT), yFast, 'b', 'LineWidth', 2);      

    % Set axis limits and labels
    xlim([1, timeSteps-initialDT]); ylim([0,1]);
    set(gca,'FontSize', 15); grid on;
    xticks(0:1e3:7e3); yticks(0:0.5:1.1);
    xticklabels(string(0:1e3:7e3)); yticklabels(string(0:0.5:1.1));
    title(titles{i_cam}, 'Interpreter', 'latex', 'FontSize', fontSize);
end
ylabel({'$\frac{V(P_{xy})\cap V(\hat{P}_{xy})}{V(P_{xy})}$', ''}, 'Interpreter','latex', 'FontSize', fontSize);

%% Plot Pt
for i_cam = 1:length(cameraTypes)
    subplot(3, 3, i_cam*3);
    camType     = cameraTypes(i_cam);
    filename    = getFileName(camType, parameters);
    data        = load(filename).Historys;
    History     = cellHistory2Arr(data, initialDT);
    
    ySet        = History.SetSLAM.Pt;
    yFast       = History.FastSLAM.Pt;
    hSet1       = plot(1:(timeSteps-initialDT), ySet, 'r', 'LineWidth', 2); hold on;
    hFast1      = plot(1:(timeSteps-initialDT), yFast, 'b', 'LineWidth', 2);      

    % Set axis limits and labels
    xlim([1, timeSteps-initialDT]); ylim([min([ySet, yFast], [], 'all'), max([ySet, yFast], [], 'all')]);
    set(gca,'FontSize', 15); grid on;
    xticks(0:1e3:7e3); yticks([min([ySet, yFast], [], 'all'), max([ySet, yFast], [], 'all')]);
    xticklabels(string(0:1e3:7e3)); yticklabels(string(rad2deg([min([ySet, yFast], [], 'all'), max([ySet, yFast], [], 'all')])));
    title(titles{i_cam}, 'Interpreter', 'latex', 'FontSize', fontSize);
end
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ax  = axes(fig2,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'$\frac{V(P_{xy})\cap V(\hat{P}_{xy})}{V(P_{xy})}$', ''}, 'Interpreter','latex', 'FontSize', fontSize);

subplot(3,1,3);
set(gca,'visible','off')
legend([hSet1, hFast1],{'Ours', 'FastSLAM'}, 'Interpreter','latex', 'NumColumns', 2, 'FontSize', fontSize*0.7, 'Position', [0.25, 0.15, 0.5,0.15]);

%% Support functions
function filename = getFileName(cameraType, pr)
    filename    = "snapshot_" + cameraType + "_eva_" + num2str(pr.e_va) + "_evr_" + num2str(pr.e_vr) + ...
                    "_eSteering_" + num2str(pr.e_steering) + "_eVelocity_" + num2str(pr.e_velocity) + ...
                    "_Lt0_" + num2str(pr.epsilon_Lt) + "_Lxy0_" + num2str(pr.epsilon_Lxy) + "_P0_" + num2str(pr.epsilon_P) + '.mat';
end

function History = cellHistory2Arr(data, initialDT)
    % remove the initialization timestep
    History.SetSLAM.Pxy     = zeros(1, length(data)-initialDT);
    History.SetSLAM.Pt      = zeros(1, length(data)-initialDT);
    History.SetSLAM.isIn    = zeros(1, length(data)-initialDT);
    History.SetSLAM.PxyOr   = zeros(1, length(data)-initialDT);
    History.FastSLAM.Pxy    = zeros(1, length(data)-initialDT);
    History.FastSLAM.Pt     = zeros(1, length(data)-initialDT);
    History.FastSLAM.isIn   = zeros(1, length(data)-initialDT);
    History.FastSLAM.PxyOr  = zeros(1, length(data)-initialDT);
    for i = 1:length(data)-initialDT % remove the initialization timestep
        History.SetSLAM.Pxy(i)      = data{i+initialDT}.SetSLAM.Pxy;
        History.SetSLAM.Pt(i)       = data{i+initialDT}.SetSLAM.Pt;
        History.SetSLAM.isIn(i)     = data{i+initialDT}.SetSLAM.isIn;
        History.SetSLAM.PxyOr(i)    = data{i+initialDT}.SetSLAM.PxyOr;
        History.FastSLAM.Pxy(i)     = data{i+initialDT}.FastSLAM.Pxy;
        History.FastSLAM.Pt(i)      = data{i+initialDT}.FastSLAM.Pt;
        History.FastSLAM.isIn(i)    = data{i+initialDT}.FastSLAM.isIn;
        History.FastSLAM.PxyOr(i)   = data{i+initialDT}.FastSLAM.PxyOr;
    end
end

function timeSteps = getTimeSteps()
    data    = load('../Path.mat').Historys;
    timeSteps   = length(data);
end