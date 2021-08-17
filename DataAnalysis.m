% clear all; clc;
close all;
% addpath('./util')
% addpath('./set operation')
% addpath('./filtering')
% Historys  	= load('graphData/stereo_eva_0.087266_evr_0.1_ew_0.075_Lt0_0.0087266_Lxy0_0.1_P0_0.5.mat').Historys;

figure(1)
set(gcf,'color','w');
showMap(Historys{1}.pr, Historys{1}.costmap)
showSnapshots(Historys)

% this function draw the camera at initialization
function showMap(pr, costmap)
    plot(costmap, 'Inflation', 'off'); hold on;
    xlim([pr.Omega_L.inf(1) pr.Omega_L.sup(1)]);
    ylim([pr.Omega_L.inf(2) pr.Omega_L.sup(2)]);
    for i = 1:pr.m
        plot(pr.l_hat(1,i), pr.l_hat(2,i), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
        offset  = (-1)^(pr.l_hat(2,i) < pr.SpaceDim(2))*2;
        text(pr.l_hat(1,i), pr.l_hat(2,i)+offset, num2str(i), 'Color', 'Red', 'FontSize', 20)
        plot([pr.l_hat(1,i), pr.l_hat(1,i)+0.25*pr.Measurable_R*cos(pr.l_hat(3,i))],...
             [pr.l_hat(2,i), pr.l_hat(2,i)+0.25*pr.Measurable_R*sin(pr.l_hat(3,i))], 'r--');
    end    
end


% Draw trajectory and snapshots
function showSnapshots(Historys)
    trajectory      = zeros(length(Historys), 2);
    for i = 1:length(Historys)
        state               = Historys{i}.p_car;
        trajectory(i,:)     = [state(1), state(2)];
    end            
    plot(trajectory(:,1), trajectory(:,2), 'LineWidth', 3);
    idxs    = floor(linspace(1, length(Historys), 5));
    for i = 1:length(idxs)-1
        idx         = idxs(i) - mod(idxs(i), round(Historys{1}.pr.updateTime/Historys{1}.pr.simLoopDt)) + 1;
        % Plot Vehicle Body
        carPose     = Historys{idx}.p_car;
        carPose(3)  = rad2deg(Historys{idx}.p_car(3));
        helperPlotVehicle(carPose', Historys{1}.pr.carDims, 0);
        % Plot Sets
        plot(Historys{idx}.SetSLAM.Pxy); hold on;
    end
end