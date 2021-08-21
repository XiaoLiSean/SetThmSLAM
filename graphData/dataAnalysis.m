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
epsilon_Lts     = deg2rad(linspace(0.1, 10, num));
epsilon_Lxys    = linspace(0.01, 0.5, num);
epsilon_Ps      = linspace(0.1, 5, num);
e_mesh          = {e_vas, e_vrs, epsilon_Lts, epsilon_Lxys, epsilon_Ps};

%% Derived parameter
yTickLabels     = {rad2deg(e_vas), e_vrs, rad2deg(epsilon_Lts).*2, epsilon_Lxys.^2.*4, epsilon_Ps.^2.*4 };
vehicleVolume   = parameters.carLength*parameters.carWidth;
timeSteps       = getTimeSteps();
xoffset         = parameters.simLoopDt * timeSteps * 2;
labels          = ["$\epsilon^{v_a}$ [deg]", "$\epsilon^{v_r}$ [m]",...
                    "$V(L_{i,\theta}(k=0))$ [deg]",...
                    "$V(L_{i,xy}(k=0))$ [$m^2$]",...
                    "$V(P_{i}(k=0))$ [$m^2$]"];
%% Simulation Main
fig     = figure(1);
set(gcf,'color','w');
for i_cam = 1:length(cameraTypes)
    camType         = cameraTypes(i_cam);
    [z_max, z_min]  = findVolumeMaxMin(camType, e_mesh, vehicleVolume);
    for k = 1:length(e_mesh)
        pr      = {e_vas(1), e_vrs(1), epsilon_Lts(1), epsilon_Lxys(1), epsilon_Ps(1)};
        x       = e_mesh{k};
        yFast   = zeros(num, timeSteps);
        ySet   = zeros(num, timeSteps);
        if k == 2 && strcmp(camType,'mono') 
            continue
        end
        subplot(length(e_mesh), 2, 2*(k-1)+i_cam)
        for i = 1:num
            pr{k}   = e_mesh{k}(i);
            [e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P]    = deal(pr{:});
            if strcmp(camType,'mono')
                filename =  getFileName(camType, 0.075, e_va, 0.01, epsilon_Lt, epsilon_Lxy, epsilon_P);
            else
                filename =  getFileName(camType, 0.075, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
            end
            data    = load(filename).Historys;
            History = cellHistory2Arr(data);
            ySet(i,:)   = History.SetSLAM.Pxy/vehicleVolume;
            yFast(i,:)  = History.FastSLAM.Pxy/vehicleVolume;
        end
        plot(x, median(ySet, 2), 'r'); hold on;
        plot(x, median(yFast, 2), 'b');
        rSet    = (max(ySet, [], 2) - min(ySet, [], 2))/2;
        rFast   = (max(yFast, [], 2) - min(yFast, [], 2))/2;
        errorbar(x,median(ySet, 2),rSet,'ro');
        errorbar(x,median(yFast, 2),rFast,'bo');
    end
end
%% Support functions
function filename = getFileName(cameraType, e_w, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P)
    filename    = cameraType + "_eva_" + num2str(e_va) + "_evr_" + num2str(e_vr) + ...
                "_ew_" + num2str(e_w(1)) + "_Lt0_" + num2str(epsilon_Lt) + ...
                "_Lxy0_" + num2str(epsilon_Lxy) + "_P0_" + num2str(epsilon_P) + '.mat';
end

function [z_max, z_min] = findVolumeMaxMin(camType, e_mesh, vehicleVolume)
    zPxy    = [];
    zPt     = [];
    for k = 1:5
        pr      = {e_mesh{1}(1), e_mesh{2}(1), e_mesh{3}(1), e_mesh{4}(1), e_mesh{5}(1)};
        for i = 1:length(e_mesh{1})
            if k ~= 1 && i == 1
                continue
            end
            pr{k}   = e_mesh{k}(i);
            [e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P]    = deal(pr{:});            
            if strcmp(camType,'mono')
                filename =  getFileName(camType, 0.075, e_va, 0.01, epsilon_Lt, epsilon_Lxy, epsilon_P);
            else
                filename =  getFileName(camType, 0.075, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
            end
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