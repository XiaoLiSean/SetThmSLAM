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
num             = 25;
e_vas           = deg2rad(linspace(0.1, 10, num));
e_vrs           = linspace(0.01, 0.5, num);
epsilon_Ps      = linspace(0.1, 5, num);
epsilon_Lts     = deg2rad(linspace(0.1, 10, num));
epsilon_Lxys    = linspace(0.01, 0.5, num);
e_mesh          = {e_vas, e_vrs, epsilon_Ps, epsilon_Lts, epsilon_Lxys};

%% Derived parameter
yTickLabels     = {rad2deg(e_vas), e_vrs, epsilon_Ps.^2.*4, rad2deg(epsilon_Lts).*2, epsilon_Lxys.^2.*4};
vehicleVolume   = parameters.carLength*parameters.carWidth;
timeSteps       = getTimeSteps();
xoffset         = parameters.simLoopDt * timeSteps * 2;
initialDT       = 100; % timesteps used in initialization
labels          = ["$\epsilon^{v_a}$ [deg]", "$\epsilon^{v_r}$ [m]",...
                    "$V(P_{i}(k=0))$ [$m^2$]",...
                    "$V(L_{i,\theta}(k=0))$ [deg]",...
                    "$V(L_{i,xy}(k=0))$ [$m^2$]"];
fontSize        = 20; 
alpha1          = 0.25;
alpha2          = 0.12;

%% Simulation with Changing noise
fig     = figure(1);
set(gcf,'color','w');
set(gcf,'Position',[0 0 1000 400])
for i_cam = 1:length(cameraTypes)
    camType         = cameraTypes(i_cam);
    for k = 1:2
        pr      = {e_vas(1), e_vrs(1), epsilon_Ps(1), epsilon_Lts(1), epsilon_Lxys(1)};
        x       = e_mesh{k};
        yFast   = zeros(num, timeSteps-initialDT);
        ySet   = zeros(num, timeSteps-initialDT);
        if k == 2 && strcmp(camType,'mono')
            leg = legend([h1S, h2S, h3S, h1F, h2F, h3F],{'mean (Ours)', 'std (Ours)', 'Max/Min (Ours)', 'mean (FastSLAM)', 'std (FastSLAM)', 'Max/Min (FastSLAM)'},...
                'Interpreter','latex', 'NumColumns', 2, 'FontSize', fontSize*0.7, 'Position', [0.1, 0.2, 0.4,0.2]);
            continue
        end
        subplot(2, 2, 2*(k-1)+i_cam);
        for i = 1:num
            pr{k}   = e_mesh{k}(i);
            [e_va, e_vr, epsilon_P, epsilon_Lt, epsilon_Lxy]    = deal(pr{:});
            if strcmp(camType,'mono')
                filename =  getFileName(camType, 0.075, e_va, 0.01, epsilon_Lt, epsilon_Lxy, epsilon_P);
            else
                filename =  getFileName(camType, 0.075, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
            end
            data    = load(filename).Historys;
            History = cellHistory2Arr(data, initialDT);
            ySet(i,:)   = History.SetSLAM.Pxy/vehicleVolume;
            yFast(i,:)  = History.FastSLAM.Pxy/vehicleVolume;
        end
        mSet    = mean(ySet, 2);
        mFast   = mean(yFast, 2);
        sSet    = std(ySet, 0, 2);
        sFast   = std(yFast, 0, 2);
        h1S     = plot(x, mSet, 'r', 'LineWidth', 1); hold on;
        h1F     = plot(x, mFast, 'b', 'LineWidth', 1);
        % draw shaded area
        patchX  = [x,flip(x)];
        setPy   = [mSet-sSet; flip(mSet+sSet)]';       
        FastPy  = [mFast-sFast; flip(mFast+sFast)]';
        h2S     = patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1); 
        h2F     = patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1);
        h3S     = patch(patchX, [min(ySet,[],2); flip(max(ySet,[],2))]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2); 
        h3F     = patch(patchX, [min(yFast,[],2); flip(max(yFast,[],2))]', 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2);
        % Set axis limits and labels
        xlim([min(x), max(x)]); ylim([min([ySet, yFast], [], 'all'), max([ySet, yFast], [], 'all')]);
        set(gca,'FontSize', fontSize, 'FontName','times'); % change ticks label font size
        xlabel(labels(k), 'Interpreter', 'latex', 'FontSize', fontSize); 
        xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6));
        xticklabels(string(round(linspace(yTickLabels{k}(1), yTickLabels{k}(end), 6)*1)/1));
    end
end
ax  = axes(fig,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'$V(P_{xy})/(c_w\cdot c_l)$', ''}, 'Interpreter','latex', 'FontSize', fontSize);
currentFigure   = gcf;
title(currentFigure.Children(end), 'Monocular Camera', 'Interpreter', 'latex', 'FontSize', fontSize);
title(currentFigure.Children(3), 'Stereo Camera', 'Interpreter', 'latex', 'FontSize', fontSize);
saveas(gcf,'pxyVersusParamsA','epsc')

%% Simulation with Changing initialization
fig     = figure(2);
set(gcf,'color','w');
set(gcf,'Position',[0 0 1000 800])
for i_cam = 1:length(cameraTypes)
    camType         = cameraTypes(i_cam);
    for k = 3:length(e_mesh)
        pr      = {e_vas(1), e_vrs(1), epsilon_Ps(1), epsilon_Lts(1), epsilon_Lxys(1)};
        x       = e_mesh{k};
        yFast   = zeros(num, timeSteps-initialDT);
        ySet   = zeros(num, timeSteps-initialDT);
        if k == 2 && strcmp(camType,'mono') 
            continue
        end
        subplot(4, 2, 2*(k-2)+i_cam);
        for i = 1:num
            pr{k}   = e_mesh{k}(i);
            [e_va, e_vr, epsilon_P, epsilon_Lt, epsilon_Lxy]    = deal(pr{:});
            if strcmp(camType,'mono')
                filename =  getFileName(camType, 0.075, e_va, 0.01, epsilon_Lt, epsilon_Lxy, epsilon_P);
            else
                filename =  getFileName(camType, 0.075, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
            end
            data    = load(filename).Historys;
            History = cellHistory2Arr(data, initialDT);
            ySet(i,:)   = History.SetSLAM.Pxy/vehicleVolume;
            yFast(i,:)  = History.FastSLAM.Pxy/vehicleVolume;
        end
        mSet    = mean(ySet, 2);
        mFast   = mean(yFast, 2);
        sSet    = std(ySet, 0, 2);
        sFast   = std(yFast, 0, 2);
        plot(x, mSet, 'r', 'LineWidth', 1); hold on;
        plot(x, mFast, 'b', 'LineWidth', 1);
        % draw shaded area
        patchX  = [x,flip(x)];
        setPy   = [mSet-sSet; flip(mSet+sSet)]';       
        FastPy  = [mFast-sFast; flip(mFast+sFast)]';
        patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1); 
        patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1);
        patch(patchX, [min(ySet,[],2); flip(max(ySet,[],2))]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2); 
        patch(patchX, [min(yFast,[],2); flip(max(yFast,[],2))]', 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2);
        % Set axis limits and labels
        xlim([min(x), max(x)]); ylim([min([ySet, yFast], [], 'all'), max([ySet, yFast], [], 'all')]);
        set(gca,'FontSize', fontSize, 'FontName','times'); % change ticks label font size
        xlabel(labels(k), 'Interpreter', 'latex', 'FontSize', fontSize); 
        xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6));
        xticklabels(string(round(linspace(yTickLabels{k}(1), yTickLabels{k}(end), 6)*1)/1));
        if k == 3
            subplot(4, 2, i_cam);
            plot(x, mSet, 'r', 'LineWidth', 1); hold on;
            patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1);            
            patch(patchX, [min(ySet,[],2); flip(max(ySet,[],2))]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2); 
            xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6));
            xticklabels(string(round(linspace(yTickLabels{k}(1), yTickLabels{k}(end), 6)*1)/1));
            xlim([min(x), max(x)]);
            xlabel(labels(k), 'Interpreter', 'latex', 'FontSize', fontSize); 
            set(gca,'FontSize', fontSize, 'FontName','times');
        end
    end
end
ax  = axes(fig,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'$V(P_{xy})/(c_w\cdot c_l)$', ''}, 'Interpreter','latex', 'FontSize', fontSize);
currentFigure   = gcf;
title(currentFigure.Children(8), 'Monocular Camera', 'Interpreter', 'latex', 'FontSize', fontSize);
title(currentFigure.Children(4), 'Stereo Camera', 'Interpreter', 'latex', 'FontSize', fontSize);
annotation('rectangle',[0.12, 0.59, 0.35, 0.03],'Color','red')
annotation('rectangle',[0.56, 0.59, 0.35, 0.03],'Color','red')
annotation('arrow', [0.15,0.15],[0.62, 0.82],'Color','red')
annotation('arrow', [0.6,0.6],[0.62, 0.82],'Color','red')
saveas(gcf,'pxyVersusParamsB','epsc')

%% Support functions
function filename = getFileName(cameraType, e_w, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P)
    filename    = cameraType + "_eva_" + num2str(e_va) + "_evr_" + num2str(e_vr) + ...
                "_ew_" + num2str(e_w(1)) + "_Lt0_" + num2str(epsilon_Lt) + ...
                "_Lxy0_" + num2str(epsilon_Lxy) + "_P0_" + num2str(epsilon_P) + '.mat';
end

function History = cellHistory2Arr(data, initialDT)
    % remove the initialization timestep
    History.SetSLAM.Pxy     = zeros(1, length(data)-initialDT);
    History.SetSLAM.Pt      = zeros(1, length(data)-initialDT);
    History.SetSLAM.isIn    = zeros(1, length(data)-initialDT);
    History.FastSLAM.Pxy    = zeros(1, length(data)-initialDT);
    History.FastSLAM.Pt     = zeros(1, length(data)-initialDT);
    History.FastSLAM.isIn   = zeros(1, length(data)-initialDT);
    for i = 1:length(data)-initialDT % remove the initialization timestep
        History.SetSLAM.Pxy(i)      = data{i+initialDT}.SetSLAM.Pxy;
        History.SetSLAM.Pt(i)       = data{i+initialDT}.SetSLAM.Pt;
        History.SetSLAM.isIn(i)     = data{i+initialDT}.SetSLAM.isIn;
        History.FastSLAM.Pxy(i)     = data{i+initialDT}.FastSLAM.Pxy;
        History.FastSLAM.Pt(i)      = data{i+initialDT}.FastSLAM.Pt;
        History.FastSLAM.isIn(i)    = data{i+initialDT}.FastSLAM.isIn;
    end
end

function timeSteps = getTimeSteps()
    data    = load('stereo_eva_0.13134_evr_0.01_ew_0.075_Lt0_0.0017453_Lxy0_0.01_P0_0.1.mat').Historys;
    timeSteps   = length(data);
end