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
e_vrs           = linspace(0.1, 0.5, num);
epsilon_Lts     = deg2rad(linspace(0.1, 10, num));
epsilon_Lxys    = linspace(0.1, 0.5, num);
epsilon_Ps      = linspace(0.1, 5, num);  
e_steerings     = deg2rad(linspace(0.1, 10, num));
e_velocitys     = linspace(0.05, 5, num); 
e_mesh          = {e_vas, e_vrs, epsilon_Ps, epsilon_Lts, epsilon_Lxys, e_velocitys, e_steerings};

%% Derived parameter
xTickLabels     = {rad2deg(e_vas), e_vrs, epsilon_Ps.^2.*4, rad2deg(epsilon_Lts).*2, epsilon_Lxys.^2.*4, e_velocitys, rad2deg(e_steerings)};
vehicleVolume   = parameters.carLength*parameters.carWidth;
timeSteps       = getTimeSteps();
initialDT       = parameters.updateTime/parameters.propTime; % timesteps used in initialization
updateRuns      = floor(timeSteps/initialDT);
titles          = {'Monocular Camera', 'Stereo Camera'};
labels          = ["$\epsilon^{u_1}$ [m/s]",  "$\epsilon^{u_2}$ [deg]"];
filenames       = ["sensitivityVelocity", "sensitivitySteering"];
fontSize        = 20; 
alpha1          = 0.25;
alpha2          = 0.12;

%% Simulation with Changing noise
for k = 6:7
roundFactor     = 1;
fig     = figure(k-5);
set(gcf,'color','w');
set(gcf,'Position',[0 0 1000 400])
for i_cam = 1:length(cameraTypes)
    camType = cameraTypes(i_cam);
    pr      = {e_vas(1), e_vrs(1), epsilon_Ps(1), epsilon_Lts(1), epsilon_Lxys(1), e_velocitys(1), e_steerings(1)};
    x       = e_mesh{k};
    yBFast  = zeros(num, updateRuns);
    yBSet   = zeros(num, updateRuns);
    yOFast  = zeros(num, updateRuns);
    yOSet   = zeros(num, updateRuns);

    for i = 1:num
        pr{k}       = e_mesh{k}(i);
        [e_va, e_vr, epsilon_P, epsilon_Lt, epsilon_Lxy, e_velocity, e_steering]    = deal(pr{:});
        filename    =  getFileName(camType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity);
        data        = load(filename).Historys;
        History     = sparseCellHistory2Arr(data, initialDT);

        yBSet(i,:)  = History.SetSLAM.PxyOr./History.SetSLAM.Pxy;
        yBFast(i,:) = History.FastSLAM.PxyOr./History.FastSLAM.Pxy;
        
        yOSet(i,:)  = getAngleDistances(History.SetSLAM.PtInf, History.SetSLAM.PtSup, History.theta_hat);
        yOFast(i,:) = getAngleDistances(History.FastSLAM.PtInf, History.FastSLAM.PtSup, History.theta_hat);
        %yOSet(i,:)  = History.SetSLAM.Pt;
        %yOFast(i,:) = History.FastSLAM.Pt;
    end
    [mBSet, sBset]      = deal(mean(yBSet, 2), std(yBSet, 0, 2));
    [mOSet, sOset]      = deal(mean(yOSet, 2), std(yOSet, 0, 2));
    [mBFast, sBFast]    = deal(mean(yBFast, 2), std(yBFast, 0, 2));
    [mOFast, sOFast]    = deal(mean(yOFast, 2), std(yOFast, 0, 2));

    subplot(2,2,i_cam);
    plot(x, mBSet, 'r', 'LineWidth', 1); hold on;
    plot(x, mBFast, 'b', 'LineWidth', 1);

    patchX      = [x,flip(x)];
    patch(patchX, [mBSet-sBset; flip(mBSet+sBset)]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1); 
    patch(patchX, [mBFast-sBFast; flip(mBFast+sBFast)]', 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1);
    %patch(patchX, [min(yBSet,[],2); flip(max(yBSet,[],2))]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2); 
    %patch(patchX, [min(yBFast,[],2); flip(max(yBFast,[],2))]', 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2);
    
    xlim([min(x), max(x)]); ylim([0,1]);
    set(gca,'FontSize', 15); grid on;
    xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6)); xticklabels(string(round(linspace(xTickLabels{k}(1), xTickLabels{k}(end), 6)*roundFactor)/roundFactor));
    yticks([0,1]); yticklabels([0,1]);
    ytickangle(90);
    if i_cam == 1     
        ylabel({'', '$\frac{V(P_{xy}\cap\hat{P}_{xy})}{V(P_{xy})}$'}, 'Interpreter','latex', 'FontSize', 1.5*fontSize);
    end
    title(titles{i_cam}, 'Interpreter', 'latex', 'FontSize', fontSize);

    subplot(2,2,2+i_cam);
    plot(x, mOSet, 'r', 'LineWidth', 1); hold on;
    plot(x, mOFast, 'b', 'LineWidth', 1);

    patchX      = [x,flip(x)];
    patch(patchX, [mOSet-sOset; flip(mOSet+sOset)]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1); 
    patch(patchX, [mOFast-sOFast; flip(mOFast+sOFast)]', 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha1, 'EdgeAlpha',alpha1);
    %patch(patchX, [min(yOSet,[],2); flip(max(yOSet,[],2))]', 'red', 'EdgeColor','red', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2); 
    %patch(patchX, [min(yOFast,[],2); flip(max(yOFast,[],2))]', 'blue', 'EdgeColor','blue', 'FaceAlpha',alpha2, 'EdgeAlpha',alpha2);

    max_temp    = rad2deg(max([mOSet+sOset, mOFast+sOFast], [], 'all'));
    yTickMax    = ceil(max_temp/10)*10;
    xlim([min(x), max(x)]); ylim([0, deg2rad(yTickMax)]);
    set(gca,'FontSize', 15); grid on;
    xticks(linspace(e_mesh{k}(1), e_mesh{k}(end), 6)); xticklabels(string(round(linspace(xTickLabels{k}(1), xTickLabels{k}(end), 6)*roundFactor)/roundFactor));
    yticks([0, deg2rad(yTickMax)]); yticklabels([0, yTickMax]);
    ytickangle(90);
    xlabel(labels(k-5), 'Interpreter', 'latex', 'FontSize', fontSize); 
    if i_cam == 1      
        ylabel({'$|sup P_{\theta}-\hat{p}_{\theta}|$', '$+|inf P_{\theta}-\hat{p}_{\theta}|$', '[deg]'}, 'Interpreter','latex', 'FontSize', fontSize);
    end
end
saveas(gcf,filenames(k-5),'epsc')
end

%% Support functions
function dThetas = getAngleDistances(Infs, Sups, angles)
    dThetas  = zeros(1,length(angles));
    for i = 1:length(angles)
        dThetas(i)  = getAngleDistance(Infs(i), Sups(i), angles(i));
    end
end

function dtheta = getAngleDistance(inf, sup, angle)
    Pt  = interval(inf, sup);
    if in(Pt, wrapToPi(angle)) || in(Pt, wrapTo2Pi(angle))
        dtheta  = sup - inf;
    else
        offset  = min([getMinDistance(inf, angle), getMinDistance(sup, angle)]);
        dtheta  = 2*offset + sup - inf;
    end
    if dtheta > 2*pi
        error('single span larger than 2pi')
    end
end

function dmin = getMinDistance(t1, t2)
    d1  = abs(wrapToPi(t1) - wrapTo2Pi(t2));
    d2  = abs(wrapToPi(t1) - wrapToPi(t2));
    d3  = abs(wrapTo2Pi(t1) - wrapTo2Pi(t2));
    d4  = abs(wrapTo2Pi(t1) - wrapToPi(t2));
    dmin    = min([d1,d2,d3,d4]);
end

function filename = getFileName(cameraType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity)
    filename    = "sensitivity/" + cameraType + "_eva_" + num2str(e_va) + "_evr_" + num2str(e_vr) + ...
                    "_eSteering_" + num2str(e_steering) + "_eVelocity_" + num2str(e_velocity) + ...
                    "_Lt0_" + num2str(epsilon_Lt) + "_Lxy0_" + num2str(epsilon_Lxy) + "_P0_" + num2str(epsilon_P) + '.mat';
end

function History = sparseCellHistory2Arr(data, initialDT)
    History.theta_hat       = zeros(1, floor(length(data)/initialDT));
    % remove the initialization timestep    
    History.SetSLAM.Pxy     = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.Pt      = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.PtInf   = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.PtSup   = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.isIn    = zeros(1, floor(length(data)/initialDT));
    History.SetSLAM.PxyOr   = zeros(1, floor(length(data)/initialDT));
    
    History.FastSLAM.Pxy    = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.Pt     = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.PtInf  = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.PtSup  = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.isIn   = zeros(1, floor(length(data)/initialDT));
    History.FastSLAM.PxyOr  = zeros(1, floor(length(data)/initialDT));
    for i = 1:floor(length(data)/initialDT) % remove the initialization timestep    
        History.theta_hat(i)        = data{i*initialDT+1}.theta_car;   
        
        History.SetSLAM.Pxy(i)      = data{i*initialDT+1}.SetSLAM.Pxy;
        History.SetSLAM.Pt(i)       = data{i*initialDT+1}.SetSLAM.Pt;
        History.SetSLAM.PtInf(i)    = data{i*initialDT+1}.SetSLAM.PtInf;
        History.SetSLAM.PtSup(i)    = data{i*initialDT+1}.SetSLAM.PtSup;
        History.SetSLAM.isIn(i)     = data{i*initialDT+1}.SetSLAM.isIn;
        History.SetSLAM.PxyOr(i)    = data{i*initialDT+1}.SetSLAM.PxyOr;
        
        History.FastSLAM.Pxy(i)     = data{i*initialDT+1}.FastSLAM.Pxy;
        History.FastSLAM.Pt(i)      = data{i*initialDT+1}.FastSLAM.Pt;
        History.FastSLAM.PtInf(i)   = data{i*initialDT+1}.FastSLAM.PtInf;
        History.FastSLAM.PtSup(i)   = data{i*initialDT+1}.FastSLAM.PtSup;
        History.FastSLAM.isIn(i)    = data{i*initialDT+1}.FastSLAM.isIn;
        History.FastSLAM.PxyOr(i)   = data{i*initialDT+1}.FastSLAM.PxyOr;
    end
end

function timeSteps = getTimeSteps()
    data    = load('../Path.mat').Historys;
    timeSteps   = length(data);
end