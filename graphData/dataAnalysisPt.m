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
num             = 9;
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
                
%% Simulation Main
fig     = figure(1);
set(gcf,'color','w');
for i_cam = 1:length(cameraTypes)
    camType         = cameraTypes(i_cam);
    for k = 1:length(e_mesh)
        pr      = {e_vas(1), e_vrs(1), epsilon_Ps(1), epsilon_Lts(1), epsilon_Lxys(1)};
        x       = e_mesh{k};
        yFast   = zeros(num, timeSteps-initialDT);
        ySet   = zeros(num, timeSteps-initialDT);
        if k == 2 && strcmp(camType,'mono') 
            continue
        end
        subax   = subplot(length(e_mesh), 2, 2*(k-1)+i_cam);
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
            ySet(i,:)   = History.SetSLAM.Pt;
            yFast(i,:)  = History.FastSLAM.Pt;
        end
        cSet    = (max(ySet, [], 2) + min(ySet, [], 2))/2;
        cFast   = (max(yFast, [], 2) + min(yFast, [], 2))/2;
        rSet    = (max(ySet, [], 2) - min(ySet, [], 2))/2;
        rFast   = (max(yFast, [], 2) - min(yFast, [], 2))/2; 
        plot(x, cSet, 'r', 'LineWidth', 1); hold on;
        plot(x, cFast, 'b', 'LineWidth', 1);
        % draw shaded area
        patchX  = [x,flip(x)];
        setPy   = [cSet-rSet; flip(cSet+rSet)]';       
        FastPy  = [cFast-rFast; flip(cFast+rFast)]';
        patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha',.2, 'EdgeAlpha',.4); 
        patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha',.2, 'EdgeAlpha',.4);
        % Set axis limits and labels
        yMin    = min([ySet, yFast], [], 'all');
        yMax    = max([ySet, yFast], [], 'all');
        xlim([min(x), max(x)]); ylim([yMin, yMax]);
        set(gca,'FontSize', 15); % change ticks label font size
        xlabel(labels(k), 'Interpreter', 'latex', 'FontSize', 25); 
        xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6));
        xticklabels(string(round(linspace(yTickLabels{k}(1), yTickLabels{k}(end), 6)*100)/100));
        yticks(linspace(yMin, yMax, 3));
        yticklabels(string(round(rad2deg(linspace(yMin, yMax, 3))*1)/1));
        if k == 3
            figure(2);
            set(gcf,'color','w');
            subplot(2,1,i_cam);
            plot(x, cSet, 'r', 'LineWidth', 1); hold on;
            patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha',.2, 'EdgeAlpha',.4); 
            xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6));
            xticklabels(string(round(linspace(yTickLabels{k}(1), yTickLabels{k}(end), 6)*100)/100));
            yticks(linspace(min(ySet, [], 'all'), max(ySet, [], 'all'), 3));
            yticklabels(string(round(rad2deg(linspace(min(ySet, [], 'all'), max(ySet, [], 'all'), 3))*1)/1));
            set(gca,'FontSize', 15);
            figure(1);
        end
    end
end
ax  = axes(fig,'visible','off');
ax.YLabel.Visible   ='on';
ylabel(ax, {'$V(P_{t})$ [deg]', ''}, 'Interpreter','latex', 'FontSize', 25);
currentFigure   = gcf;
title(currentFigure.Children(end), 'Monocular Camera', 'Interpreter', 'latex', 'FontSize', 25);
title(currentFigure.Children(6), 'Stereo Camera', 'Interpreter', 'latex', 'FontSize', 25);

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