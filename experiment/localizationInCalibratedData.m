% this program is used to test the localization algorithm in the recorded
% data after python calibration. 
close all; clc;
clear all;
addpath('../set operation')
addpath('../util')

%% Main
figure(1);
set(gcf,'color','w');
set(gcf,'Position',[0 0 900 900])
pause(5)
pr          = params;
SetSLAM     = SetThmSLAM(pr, true, false, false, false, []);
fid         = fopen('calibrationData/calibratedData.txt','rt');
is_initial  = true;

%% plot the field and lidars
plot(pr.Omega_L); hold on;
leg             = imread('legend.png');
len_i           = size(leg,2)/1500;
wid_i           = size(leg,1)/1500;
imagesc([pr.Omega_L.inf(1), pr.Omega_L.inf(1)+len_i], [pr.Omega_L.sup(2), pr.Omega_L.sup(2)-wid_i], leg);
lidar           = imread('rplidar.png');
Measurable_R    = 1; 
lidar_resol     = 2500;
for i = 1:pr.m
    lidar_i = imrotate_white(lidar, -rad2deg(pr.l_hat(3,i)));
    len_i   = size(lidar_i,2)/lidar_resol;
    wid_i   = size(lidar_i,1)/lidar_resol;
    imagesc([pr.l_hat(1,i)-0.5*len_i, pr.l_hat(1,i)+0.5*len_i], [pr.l_hat(2,i)-0.5*wid_i, pr.l_hat(2,i)+0.5*wid_i], lidar_i);
    hlxy    = plot(pr.l_hat(1,i), pr.l_hat(2,i), 'r.', 'MarkerSize', 15, 'LineWidth', 2);
    hlt     = plot([pr.l_hat(1,i), pr.l_hat(1,i)+0.25*Measurable_R*cos(pr.l_hat(3,i))],...
                [pr.l_hat(2,i), pr.l_hat(2,i)+0.25*Measurable_R*sin(pr.l_hat(3,i))], 'r--', 'LineWidth', 2);
end
plot(pr.Omega_L); 
pause(5)
%% Simulation main
currentStep = 0;
plotT0      = inf;
plotTf      = inf;
plotDStep   = 5;
trajectory  = [];
unitP       = decomposeCirc2ConvPolygons([0,0], 0.12, 12);
tic
while true
    % read single data at a certain timestamp from recorded scream
    newline     = fgetl(fid);
    if ~ischar(newline)
      break; 
    end
    data    = split(newline, ',');
    if length(data) <= 6
        continue
    end
    if is_initial
        dt      = 0;
        p_prev  = [str2double(data{2}); str2double(data{3})];
        is_initial  = false;
    else
        dt      = str2double(data{1}) - t_prev;
        p_prev  = p_hat;
        if dt < 0
            error('negative dt');
        end
    end
    if currentStep > plotTf
        break
    end
    t_prev  = str2double(data{1});
    p_hat   = [str2double(data{2}); str2double(data{3})];
    % SetSLAM localization
    distance    = dt*pr.maxSpeed;
	SetSLAM.propagateSetsWithDistance(distance);
    [Mas, Mrs, A_hats, Ma, Mr]  = formulateMeasurements(data, pr.m);
    for i = 1:length(Mas)
        SetSLAM.getMeasureAndMatching(Mas{i}, Mrs{i}, A_hats{i});
        SetSLAM.updateSets();
    end
    % visualization
    if currentStep > plotT0 && currentStep < plotTf
        trajectory  = [trajectory; p_hat(1), p_hat(2)];
        plot(trajectory(:,1), trajectory(:,2), 'k--', 'LineWidth', 1.5);
    end
    if dt ~= 0
        if (mod(currentStep, plotDStep) < plotDStep - 1e-4) && (mod(currentStep, plotDStep) > 1e-4) ||...
                (currentStep < plotT0 || currentStep > plotTf) 
            delete(h1);
            delete(h2);
            delete(h3);
        end
    end
    h1      = plot(p_hat(1), p_hat(2), 'r.', 'MarkerSize', 15);
    overP   = plus(SetSLAM.P{1}, unitP);
    h2      = plot(overP, [1,2], 'LineWidth', 2, 'Color', 'cyan');
    h3      = circles(p_hat(1), p_hat(2), 0.12,  'LineWidth', 2, 'EdgeColor', 'k', 'FaceColor', 'k', 'FaceAlpha', 0.3);
    set(gca,'FontSize', 25, 'FontName', 'times'); % change ticks label font size
    axis equal; grid on; xlim([pr.Omega_L.inf(1),pr.Omega_L.sup(1)]); ylim([pr.Omega_L.inf(2),pr.Omega_L.sup(2)]);
    % check_feasibility(p_hat, p_prev, distance, pr, Ma, Mr)
    if in(SetSLAM.P{1}, p_hat) == 0
        error('nominal state outside the set');
    end
    currentStep     = currentStep + 1;
    pause(0.01)
end
fclose(fid);
%legend([h1, hlt, h2, h3],{'lidar/marker nominal position $\hat{l}_{xy}$, $\hat{p}_{i}$ ', 'lidar nominal orientation $\hat{l}_{\theta}$ ',...
%    'estimated robot body $P_{xy}$ ', 'robot body '}, 'Interpreter', 'latex', 'FontSize', 13, 'NumColumns', 1, 'Location', 'northwest');
toc
%% Sub function used to pre-process incoming data
function [Mas, Mrs, A_hats, Ma, Mr] = formulateMeasurements(data, m)
    lidar_i     = 0;
    i           = 4;
    while true
        if i > length(data)
            break
        end
        if strcmp(data{i}, '-')
            i           = i+1;
            lidar_i     = lidar_i+1;
            Ma{lidar_i} = [];
            Mr{lidar_i} = [];
            continue
        else
            Ma{lidar_i} = [Ma{lidar_i}, str2double(data{i})];
            Mr{lidar_i} = [Mr{lidar_i}, str2double(data{i+1})];
            i           = i+2;
        end
    end
    % this part is used to ensure each tuple [Ma, Mr, A_hat] contains only
    % one measurement to the marker
    z_num   = zeros(m,1);
    for i = 1:m
        z_num(i)    = length(Ma{i});
    end
    for i = 1:max(z_num)
        for j = 1:m
            if z_num(j) > 0
                Mas{i}{j}   = Ma{j}(z_num(j));
                Mrs{i}{j}   = Mr{j}(z_num(j));
                A_hats{i}{j}= eye(1);
            else
                Mas{i}{j}   = [];
                Mrs{i}{j}   = [];
                A_hats{i}{j}= [];
            end
        end
        z_num   = z_num - 1;
    end
end
%% Check if the measurement has a bounded error
function check_feasibility(p_hat, p_prev, distance, pr, Ma, Mr)
    distance_gt     = norm(p_hat-p_prev,2);
    if distance_gt > distance
        error('Ground true displacement larger than bound');
    end
    for i = 1:pr.m
        for j = 1:length(Ma{i})
            range_gt        = norm(p_hat-[pr.l_hat(1,i);pr.l_hat(2,i)]);
            range_error     = abs(range_gt-Mr{i}(j));
            if range_error > pr.e_vr
                error('Range error exceed bound');
            end            
            bearing_gt      = atan2(p_hat(2)-pr.l_hat(2,i),p_hat(1)-pr.l_hat(1,i))-pr.l_hat(3,i);
            bearing_error   = min([mod(bearing_gt-Ma{i}(j), 2*pi), abs(bearing_gt-Ma{i}(j)), mod(abs(bearing_gt-Ma{i}(j)), 2*pi)]);
            if bearing_error > pr.e_va
                error('Bearing error exceed bound');
            end
        end
    end
end

%% Rotate images
function rotated_image = imrotate_white(image, rot_angle_degree)
    tform = affine2d([cosd(rot_angle_degree)    -sind(rot_angle_degree)     0; ...
                        sind(rot_angle_degree)     cosd(rot_angle_degree)     0; ...
                        0                          0                          1]);
    rotated_image = imwarp(image, tform, 'interp', 'cubic', 'fillvalues', 255);
end