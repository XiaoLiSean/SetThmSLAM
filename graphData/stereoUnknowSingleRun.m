clear all; clc;
close all;
addpath('..')
addpath('../util')
addpath('../set operation')
addpath('../filtering')

% Pxy Ptheta Volume versus time
%% Change Parameters
parameters      = params;
camType         = "stereo";

%% Derived parameter
timeSteps       = getTimeSteps();
timeInterval    = parameters.updateTime/parameters.propTime;
initialDT       = timeInterval;
fontSize        = 30;

%% Plot isIn
figure(1);
set(gcf,'color','w');
set(gcf,'Position',[000 000 800 300]);

filename    = getFileName(camType, parameters);
data        = load(filename).Historys;
History     = sparseCellHistory2Arr(data, initialDT);

ySet        = History.SetSLAM.isIn;
yFast       = History.FastSLAM.isIn;
hSet1       = plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2); hold on;
hFast1      = plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);      

% Set axis limits and labels
xlim([1, floor(timeSteps/initialDT)]); ylim([0,1]);
set(gca,'FontSize', 20); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([0,1]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels({'false', 'true'});
ytickangle(90);
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ylabel({'$\hat{P}_{xy}\subset P_{xy}$', 'and $\hat{p}_{\theta}\in P_{\theta}$'}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'isInSingleRun_Stereo','epsc')

%% Plot Pxy
figure(2);
set(gcf,'color','w');
set(gcf,'Position',[000 000 800 300]);

filename    = getFileName(camType, parameters);
data        = load(filename).Historys;
History     = sparseCellHistory2Arr(data, initialDT);

ySet        = History.SetSLAM.PxyOr ./ History.SetSLAM.Pxy;
yFast       = History.FastSLAM.PxyOr ./ History.FastSLAM.Pxy;
plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2); hold on;
plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);      

% Set axis limits and labels
xlim([1, floor(timeSteps/initialDT)]); ylim([0,1]);
set(gca,'FontSize', 20); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks(0:0.5:1.1);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string(0:0.5:1.1));
ytickangle(90);

xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ylabel({'$\frac{V(P_{xy}\cap\hat{P}_{xy})}{V(P_{xy})}$'}, 'Interpreter','latex', 'FontSize', 1.5*fontSize);
saveas(gcf,'pxySingleRun_Stereo_metric','epsc')

%% Plot Pt
figure(3);
set(gcf,'color','w');
set(gcf,'Position',[000 000 800 300]);
filename            = getFileName(camType, parameters);
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
hSet2   = patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25); hold on;
hFast2  = patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25);
hpt     = plot(1:floor(timeSteps/initialDT), zeros(size(History.pt)), 'r-.', 'LineWidth', 1);

xlim([1, floor(timeSteps/initialDT)]); ylim([deg2rad(-34), deg2rad(8)]);
set(gca,'FontSize', 20); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([deg2rad(-34), deg2rad(8)]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([-34, 8]));
ytickangle(90);
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ylabel({'$p_{\theta}$-$\hat{p}_{\theta}$ [deg]'}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'ptSingleRun_Stereo','epsc')

%% Plot pt metric
figure(4);
set(gcf,'color','w');
set(gcf,'Position',[000 000 800 300]);
m2Set   = abs(History.SetSLAM.PtInf-History.pt) + abs(History.SetSLAM.PtSup-History.pt);
m2Fast  = abs(History.FastSLAM.PtInf-History.pt) + abs(History.FastSLAM.PtSup-History.pt);
plot(1:floor(timeSteps/initialDT), m2Set, 'r', 'LineWidth', 2); hold on;
plot(1:floor(timeSteps/initialDT), m2Fast, 'b', 'LineWidth', 2); 

xlim([1, floor(timeSteps/initialDT)]); ylim([0, 1]);
set(gca,'FontSize', 20); grid on;
xticks(0:20:floor(timeSteps/initialDT)); yticks([0, 0.5, 1]);
xticklabels(string(0:20:floor(timeSteps/initialDT))); yticklabels(string([0, 0.5, 1]));
ytickangle(90);
xlabel('Time Steps', 'Interpreter', 'latex', 'FontSize', fontSize);
ylabel({strcat('$|$', 'max', '$P_{\theta}-\hat{p}_{\theta}|$'), strcat('$+|$', 'min', '$P_{\theta}-\hat{p}_{\theta}|$'), '[deg]'}, 'Interpreter','latex', 'FontSize', fontSize);
saveas(gcf,'ptSingleRun_Stereo_metric','epsc')

%% Legend plot
figure(5);
set(gcf,'color','w');
set(gcf,'Position',[0 0 600 400])
subplot(2,1,1);
hSet1   = plot(1:floor(timeSteps/initialDT), ySet, 'r', 'LineWidth', 2); hold on;
hFast1  = plot(1:floor(timeSteps/initialDT), yFast, 'b', 'LineWidth', 2);  
hSet2   = patch(patchX, setPy, 'red', 'EdgeColor','red', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25); hold on;
hFast2  = patch(patchX, FastPy, 'blue', 'EdgeColor','blue', 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25);
hpt     = plot(1:floor(timeSteps/initialDT), zeros(size(History.pt)), 'r-.', 'LineWidth', 1);

subplot(2,1,2);
set(gca,'visible','off')
legend([hSet1, hFast1, hSet2, hFast2, hpt],{'Ours', 'FastSLAM', '[min$P_{\theta}$-$\hat{p}_{\theta}$, max$P_{\theta}$-$\hat{p}_{\theta}$] (Ours)',...
    '[min$P_{\theta}$-$\hat{p}_{\theta}$, max$P_{\theta}$-$\hat{p}_{\theta}$] (FastSLAM)', 'nominal orientation'}, 'Interpreter','latex', 'NumColumns', 1, 'FontSize', fontSize);


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