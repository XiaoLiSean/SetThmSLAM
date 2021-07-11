% this program is used to test the localization algorithm in the recorded
% data after python calibration. 
close all; clc;
clear all;
addpath('../set operation')
addpath('../util')

%% Main
figure(1);
pr          = params;
SetSLAM     = SetThmSLAM(pr, false, true, false, false, []);
fid         = fopen('calibrationData/calibratedData.txt','rt');
is_initial  = true;

%% plot the field and lidars
plot(pr.Omega_L); hold on;
for i = 1:pr.m
    plot(pr.l_hat(1,i), pr.l_hat(2,i), 'rx', 'MarkerSize', 15);
    plot([pr.l_hat(1,i), pr.l_hat(1,i)+0.25*pr.Measurable_R*cos(pr.l_hat(3,i))],...
         [pr.l_hat(2,i), pr.l_hat(2,i)+0.25*pr.Measurable_R*sin(pr.l_hat(3,i))], 'r--');
end
%% Simulation main
while true
    % read single data at a certain timestamp from recorded scream
    newline     = fgetl(fid);
    if ~ischar(newline)
      break; 
    end
    data    = split(newline, ',');
    if length(data) < 3
        continue
    end
    if is_initial
        dt  = 0;
        is_initial  = false;
    else
        dt  = str2double(data{1}) - t_prev;
    end
    t_prev  = str2double(data{1});
    p_hat   = [str2double(data{2}); str2double(data{3})];
    % SetSLAM localization
    tic
    distance    = dt*pr.maxSpeed;
	SetSLAM.propagateSetsWithDistance(distance);
    [Mas, Mrs, A_hats]  = formulateMeasurements(data, pr.m);
    for i = 1:length(Mas)
        SetSLAM.getMeasureAndMatching(Mas{i}, Mrs{i}, A_hats{i});
        SetSLAM.updateSets();
    end
    toc
    % visualization
    if dt ~= 0
        delete(h1);
        delete(h2);
        for i = 1:pr.m
            delete(hxy{i});
            delete(ht{i});
        end
    end
    h1  = plot(p_hat(1), p_hat(2), 'r.', 'MarkerSize', 15);
    h2  = plot(SetSLAM.P{1});
    for i = 1:pr.m
        hxy{i}  = plot(SetSLAM.Lxy{i});
        t1      = SetSLAM.Lt{i}.inf;
        t2      = SetSLAM.Lt{i}.sup;
        r       = 0.25*pr.Measurable_R; % line length to visualize the heading uncertainty
        x       = [pr.l_hat(1,i)+r*cos(t1), pr.l_hat(1,i), pr.l_hat(1,i)+r*cos(t2)];
        y       = [pr.l_hat(2,i)+r*sin(t1), pr.l_hat(2,i), pr.l_hat(2,i)+r*sin(t2)];
        ht{i}   = plot(x, y, 'b');
    end
    pause(0.01)
end
fclose(fid);

%% Sub function used to pre-process incoming data
function [Mas, Mrs, A_hats] = formulateMeasurements(data, m)
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