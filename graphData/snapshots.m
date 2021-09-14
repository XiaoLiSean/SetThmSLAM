%clear all; clc;
close all;
addpath('..')
addpath('../util')
addpath('../set operation')
addpath('../filtering')
%Historys  	= load('snapshot_stereo_eva_0.087266_evr_0.1_ew_0.075_Lt0_0.087266_Lxy0_0.1_P0_0.5.mat').Historys;

figure(1)
pr      = params;
Historys{1}.pr.carDims  = pr.carDims;
set(gcf,'color','w'); 
showMap(Historys{1}.pr, Historys{1}.costmap)
showSnapshots(Historys)

% this function draw the camera at initialization
function showMap(pr, costmap)
    plot(costmap, 'Inflation', 'off'); hold on;
    xlim([pr.Omega_L.inf(1) pr.Omega_L.sup(1)]);
    ylim([pr.Omega_L.inf(2) pr.Omega_L.sup(2)]);
    for i = 1:length(pr.l_hat)
        h1  = plot(pr.l_hat(1,i), pr.l_hat(2,i), 'r.', 'MarkerSize', 20, 'LineWidth', 2); hold on;
        offset  = (-1)^(pr.l_hat(2,i) < pr.SpaceDim(2))*2;
        text(pr.l_hat(1,i), pr.l_hat(2,i)+offset, num2str(i), 'Color', 'Red', 'FontSize', 25, 'FontName','times')
        h2  = plot([pr.l_hat(1,i), pr.l_hat(1,i)+0.25*pr.Measurable_R*cos(pr.l_hat(3,i))],...
             [pr.l_hat(2,i), pr.l_hat(2,i)+0.25*pr.Measurable_R*sin(pr.l_hat(3,i))], 'r--', 'LineWidth', 2);
        if i == 1
            set(h1,{'DisplayName'},{'camera position $\hat{l}_{xy}$ '})
            set(h2,{'DisplayName'},{'camera/robot orientation $\hat{l}_{\theta}, \hat{p}_{\theta}$ '})
        else
            h1.Annotation.LegendInformation.IconDisplayStyle = 'off';
            h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    end    
end


% Draw trajectory and snapshots
function showSnapshots(Historys)
    set(gca,'FontSize', 25, 'FontName','times');
    trajectory      = zeros(length(Historys), 2);
    for i = 1:length(Historys)
        state               = Historys{i}.p_car;
        trajectory(i,:)     = [state(1), state(2)];
    end            
    plot(trajectory(:,1), trajectory(:,2), 'LineWidth', 2, 'DisplayName', 'trajectory ');
    idxs        = floor(linspace(1, length(Historys), 5));
    r       = Historys{1}.pr.carLength;
    for i = 1:length(idxs)-1
        idx         = idxs(i) - mod(idxs(i), round(Historys{1}.pr.updateTime/Historys{1}.pr.simLoopDt)) + 1;
        % Plot Vehicle Body
        carPose     = Historys{idx}.p_car;
        carPose(3)  = rad2deg(Historys{idx}.p_car(3));      
        if i == 1
            helperPlotVehicle(carPose', Historys{1}.pr.carDims, 0, 'Color', 'blue', 'DisplayName', 'trajectory snapshot ');
        else
            helperPlotVehicle(carPose', Historys{1}.pr.carDims, 0, 'Color', 'blue');
        end
        h1  = plot([Historys{idx}.pxy(1), Historys{idx}.pxy(1)+r*cos(Historys{idx}.pt)],...
            [Historys{idx}.pxy(2), Historys{idx}.pxy(2)+r*sin(Historys{idx}.pt)], 'r--', 'LineWidth', 2);
        h1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        % Plot Reconstructed Sets in SetSLAM
        t1  = Historys{idx}.SetSLAM.Pt.inf;
        t2  = Historys{idx}.SetSLAM.Pt.sup;
        x   = [Historys{idx}.pxy(1)+r*cos(t1), Historys{idx}.pxy(1), Historys{idx}.pxy(1)+r*cos(t2)];
        y   = [Historys{idx}.pxy(2)+r*sin(t1), Historys{idx}.pxy(2), Historys{idx}.pxy(2)+r*sin(t2)];
        h2  = plot(Historys{idx}.SetSLAM.Pxy, [1,2], 'b', 'LineWidth', 2);
        h3  = plot(x, y, 'b', 'LineWidth', 2);
        h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
        % Plot Reconstructed Sets in FastSLAM        
        t1  = Historys{idx}.FastSLAM.Pt.inf;
        t2  = Historys{idx}.FastSLAM.Pt.sup;
        x   = [Historys{idx}.pxy(1)+r*cos(t1), Historys{idx}.pxy(1), Historys{idx}.pxy(1)+r*cos(t2)];
        y   = [Historys{idx}.pxy(2)+r*sin(t1), Historys{idx}.pxy(2), Historys{idx}.pxy(2)+r*sin(t2)];
        h4  = plot(Historys{idx}.FastSLAM.Pxy, [1,2], 'g', 'LineWidth', 2);
        h5  = plot(x, y, 'g', 'LineWidth', 2);
        h4.Annotation.LegendInformation.IconDisplayStyle = 'off';
        if i == 1
            set(h3,{'DisplayName'},{'estimated $P_{xy}$, $P_{\theta}$ (Ours) '})
            set(h5,{'DisplayName'},{'estimated $P_{xy}$, $P_{\theta}$ (FastSLAM) '})
        else
            h3.Annotation.LegendInformation.IconDisplayStyle = 'off';
            h5.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    end
    legend('Interpreter','latex', 'Location', 'southoutside', 'NumColumns', 3, 'FontSize', 25);
end