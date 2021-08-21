clear all; clc;
close all;
addpath('..')
addpath('../util')
addpath('../set operation')
addpath('../filtering')

% Pxy Ptheta Volume versus time
%% Change Parameters
parameters      = params;
num             = 5;
e_vas           = deg2rad(linspace(0.1, 10, num));
e_vrs           = linspace(0.01, 0.5, num);
epsilon_Lts     = deg2rad(linspace(0.1, 10, num));
epsilon_Lxys    = linspace(0.01, 0.5, num);
epsilon_Ps      = linspace(0.1, 5, num);
e_mesh          = {e_vas, e_vrs, epsilon_Lts, epsilon_Lxys, epsilon_Ps};

%% Derived parameter
yTickLabels     = {rad2deg(e_vas), e_vrs, rad2deg(epsilon_Lts).*2, epsilon_Lxys.^2.*4, epsilon_Ps.^2.*4 };
vehicleVolume   = parameters.carLength*parameters.carWidth;
timeSteps       = getTimeSteps();
[z_max, z_min]  = findVolumeMaxMin(e_mesh, vehicleVolume);
xoffset         = parameters.simLoopDt * timeSteps * 2;
labels          = ["$\epsilon^{v_a}$ [deg]", "$\epsilon^{v_r}$ [m]",...
                    "$V(L_{i,\theta}(k=0))$ [deg]",...
                    "$V(L_{i,xy}(k=0))$ [$m^2$]",...
                    "$V(P_{i}(k=0))$ [$m^2$]"];
%% Simulation Main
fig     = figure(1);
set(gcf,'color','w');
for k = 1:length(e_mesh)
    pr      = {e_vas(1), e_vrs(1), epsilon_Lts(1), epsilon_Lxys(1), epsilon_Ps(1)};
    [X,Y]   = meshgrid(1:timeSteps, e_mesh{k});
    X       = X.*parameters.simLoopDt;
    ZPxyFast    = zeros(size(X));
    ZPxySet     = zeros(size(X));
    for i = 1:num
        pr{k}   = e_mesh{k}(i);
        [e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P]    = deal(pr{:});
        filename =  getFileName('stereo', 0.075, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
        data    = load(filename).Historys;
        History = cellHistory2Arr(data);
        ZPxySet(i,:)    = History.SetSLAM.Pxy/vehicleVolume;
        ZPxyFast(i,:)   = History.FastSLAM.Pxy/vehicleVolume;              
    end
    ax_k    = subplot(length(e_mesh),1,k);
    surf(X,Y,ZPxySet); hold on; 
    surf(X+xoffset,Y,ZPxyFast); hold on;
    xlim([0, xoffset + timeSteps*parameters.simLoopDt]); ylim([e_mesh{k}(1), e_mesh{k}(end)]); 
    zMaxMin     = [min([ZPxySet;ZPxyFast], [], 'all'), max([ZPxySet;ZPxyFast], [], 'all')];
    zlim(zMaxMin);
    shading interp; colormap parula; caxis manual; caxis([z_min.Pxy, z_max.Pxy]); view(19.6283, 55.9675)
    % Plot plane of volume fraction = 1
    Xplane  = [X, X+xoffset];
    Yplane  = [Y, Y];
    Zplane  = ones(size(Xplane));
    plane   = surf(Xplane,Yplane,Zplane, 'FaceColor', 'r', 'FaceAlpha',0.5, 'EdgeColor', 'none'); hold off;
    % Specify the axis ticks and labels
    xticks([linspace(0, 60, 7), xoffset+linspace(0, 60, 7)]);
    xticklabels(string([linspace(0, 60, 7), linspace(0, 60, 7)]));
    yticks(linspace(e_mesh{k}(1), e_mesh{k}(end), num));
    yticklabels(string(linspace(yTickLabels{k}(1), yTickLabels{k}(end), num)));
    zticks([zMaxMin(1), 1, zMaxMin(2)]);
    zticklabels(round([zMaxMin(1), 1, zMaxMin(2)]*100)./100);
    % Set red ticks
    ax_k.ZTickLabel{2}  = ['\color{red}' ax_k.ZTickLabel{2}];
    set(gca,'FontSize', 15); % change ticks label font size
    ylabel(labels(k), 'Interpreter', 'latex', 'FontSize', 25); 
end
ax  = axes(fig,'visible','off');
ax.XLabel.Visible   ='on';
ax.YLabel.Visible   ='on';
xlabel(ax, 'Time [sec]', 'Interpreter','latex', 'FontSize', 25);
ylabel(ax, {'$V(P_{xy})/(c_w\cdot c_l)$ [$m^2$]', ''}, 'Interpreter','latex', 'FontSize', 25);
h   = colorbar('northoutside');
caxis([z_min.Pxy, z_max.Pxy]);
set(h, 'Position', [.13 .95 .75 .02]);

%% Support functions
function filename = getFileName(cameraType, e_w, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P)
    filename    = cameraType + "_eva_" + num2str(e_va) + "_evr_" + num2str(e_vr) + ...
                "_ew_" + num2str(e_w(1)) + "_Lt0_" + num2str(epsilon_Lt) + ...
                "_Lxy0_" + num2str(epsilon_Lxy) + "_P0_" + num2str(epsilon_P) + '.mat';
end

function [z_max, z_min] = findVolumeMaxMin(e_mesh, vehicleVolume)
    zPxy    = [];
    zPt     = [];
    for k = 1:length(e_mesh)
        pr      = {e_mesh{1}(1), e_mesh{2}(1), e_mesh{3}(1), e_mesh{4}(1), e_mesh{5}(1)};
        for i = 1:length(e_mesh{1})
            if k ~= 1 && i == 1
                continue
            end
            pr{k}   = e_mesh{k}(i);
            [e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P]    = deal(pr{:});
            filename =  getFileName('stereo', 0.075, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
            data    = load(filename).Historys;
            History = cellHistory2Arr(data);
            zPxy    = [zPxy, History.SetSLAM.Pxy/vehicleVolume];
            zPxy    = [zPxy, History.FastSLAM.Pxy/vehicleVolume];
            zPt     = [zPt, History.SetSLAM.Pt];
            zPt     = [zPt, History.FastSLAM.Pt];   
        end
    end
    z_max.Pxy   = max(zPxy);
    z_max.Pt    = max(zPt);
    z_min.Pxy   = min(zPxy);
    z_min.Pt    = min(zPt);
end

function History = cellHistory2Arr(data)
    History.SetSLAM.Pxy     = zeros(size(data));
    History.SetSLAM.Pt      = zeros(size(data));
    History.SetSLAM.isIn    = zeros(size(data));
    History.FastSLAM.Pxy    = zeros(size(data));
    History.FastSLAM.Pt     = zeros(size(data));
    History.FastSLAM.isIn   = zeros(size(data));
    for i = 1:length(data)
        History.SetSLAM.Pxy(i)      = data{i}.SetSLAM.Pxy;
        History.SetSLAM.Pt(i)       = data{i}.SetSLAM.Pt;
        History.SetSLAM.isIn(i)     = data{i}.SetSLAM.isIn;
        History.FastSLAM.Pxy(i)     = data{i}.FastSLAM.Pxy;
        History.FastSLAM.Pt(i)      = data{i}.FastSLAM.Pt;
        History.FastSLAM.isIn(i)    = data{i}.FastSLAM.isIn;
    end
end

function timeSteps = getTimeSteps()
    data    = load('stereo_eva_0.13134_evr_0.01_ew_0.075_Lt0_0.0017453_Lxy0_0.01_P0_0.1.mat').Historys;
    timeSteps   = length(data);
end