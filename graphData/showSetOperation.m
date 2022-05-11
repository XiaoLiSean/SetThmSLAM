%clear all; clc;
%close all;
addpath('..')
addpath('../util')
addpath('../set operation')
addpath('../filtering')
Historys  	= load('snapshot_stereo_eva_0.087266_evr_0.1_ew_0.075_Lt0_0.087266_Lxy0_0.1_P0_0.5.mat').Historys;

figure(1)
pr      = params;
set(gcf,'color','w'); 
idxs    = floor(linspace(1, length(Historys), 5));
r       = Historys{1}.pr.carLength;
snapshoot_i     = 3;
idx         = idxs(snapshoot_i) - mod(idxs(snapshoot_i), round(Historys{1}.pr.updateTime/Historys{1}.pr.simLoopDt)) + 1;
% Plot Vehicle Body
carPose     = Historys{idx}.p_car;
carPose(3)  = rad2deg(Historys{idx}.p_car(3));      
helperPlotVehicle(carPose', pr.carDims, 0, 'Color', 'blue', 'DisplayName', 'robot body $\hat{P}_{xy}$'); hold on;
h1  = plot([Historys{idx}.pxy(1), Historys{idx}.pxy(1)+r*cos(Historys{idx}.pt)],...
        [Historys{idx}.pxy(2), Historys{idx}.pxy(2)+r*sin(Historys{idx}.pt)], 'r--', 'LineWidth', 3);
set(h1,{'DisplayName'},{'robot orientation $\hat{p}_{\theta}$'})
% Plot Reconstructed Sets in SetSLAM
t1  = Historys{idx}.SetSLAM.Pt.inf;
t2  = Historys{idx}.SetSLAM.Pt.sup;
x   = [Historys{idx}.pxy(1)+r*cos(t1), Historys{idx}.pxy(1), Historys{idx}.pxy(1)+r*cos(t2)];
y   = [Historys{idx}.pxy(2)+r*sin(t1), Historys{idx}.pxy(2), Historys{idx}.pxy(2)+r*sin(t2)];
for i = 1:pr.n
    h2  = plot(Historys{idx}.SetSLAM.P{i}, [1,2], 'g', 'LineWidth', 3);
    h3  = plot(Historys{idx}.p_hat{i}(1), Historys{idx}.p_hat{i}(2), 'r.', 'MarkerSize', 20);        
    if i == 1
        set(h2,{'DisplayName'},{'uncertainty set $P_{i}$ of $i^{th}$ marker'})
        set(h3,{'DisplayName'},{'actual position $\hat{p}_{i}$ of $i^{th}$ marker'})
    else
        h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
        h3.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end
h4  = plot(Historys{idx}.SetSLAM.Pxy, [1,2], 'b--', 'LineWidth', 3);
h5  = plot(x, y, 'b--', 'LineWidth', 3);
set(h4,{'DisplayName'},{'estimated uncertainty sets $P_{xy}$, $P_{\theta}$'})
h5.Annotation.LegendInformation.IconDisplayStyle = 'off';
axis equal; grid on;
set(gca,'visible','off')
set(gca,'FontSize', 25);
set(gca,'xticklabels',[]);
set(gca,'yticklabels',[]);
legend('Interpreter','latex', 'Location', 'eastoutside', 'NumColumns', 1, 'FontSize', 25);