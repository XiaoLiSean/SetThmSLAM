% this program is used to test the localization algorithm in the recorded
% data after python calibration. 
close all; clc;
clear all;
addpath('../set operation')
addpath('../util')

%% Main
figure(1);
pr      = params;
SetSLAM = SetThmSLAM(pr, true, false, false, []);
fid     = fopen('calibrationData/calibratedData.txt','rt');
t_prev  = 0;
% plot the field
plot(pr.Omega_L); hold on;
plot(pr.l_hat(1), pr.l_hat(2), 'rx', 'MarkerSize', 15);
plot([pr.l_hat(1), pr.l_hat(1)+0.25*pr.Measurable_R*cos(pr.l_hat(3))],...
     [pr.l_hat(2), pr.l_hat(2)+0.25*pr.Measurable_R*sin(pr.l_hat(3))], 'r--');
while true
    % read single data at a certain timestamp from recorded scream
    newline     = fgetl(fid);
    if ~ischar(newline)
      break; 
    end
    data    = split(newline, ',');
    dt      = str2double(data{1}) - t_prev;
    p_hat   = [str2double(data{2}); str2double(data{3})];
    bearing = str2double(data{4});
    range   = str2double(data{5});
    % SetSLAM localization
    distance    = dt*pr.maxSpeed; 
	SetSLAM.propagateSetsWithDistance(distance);
    [Ma, Mr, A_hat]     = formulateMeasurements(bearing, range);
    SetSLAM.getMeasureAndMatching(Ma, Mr, A_hat);
    SetSLAM.updateSets();
    % visualization
    if dt ~= 0
        delete(h1);
        delete(h2);
        delete(h3);
        delete(h4);
    end
    h1  = plot(p_hat(1), p_hat(2), 'r.', 'MarkerSize', 15);
    h2  = plot(SetSLAM.P{1});
    h3  = plot(SetSLAM.Lxy{1});
    t1  = SetSLAM.Lt{1}.inf;
    t2  = SetSLAM.Lt{1}.sup;
    r   = 0.25*pr.Measurable_R; % line length to visualize the heading uncertainty
    x   = [pr.l_hat(1)+r*cos(t1), pr.l_hat(1), pr.l_hat(1)+r*cos(t2)];
    y   = [pr.l_hat(2)+r*sin(t1), pr.l_hat(2), pr.l_hat(2)+r*sin(t2)];
    h4  = plot(x, y, 'b');
    pause(0.01)
end
fclose(fid);

%% Sub function
function [Ma, Mr, A_hat] = formulateMeasurements(bearing, range)
    Ma{1}       = [bearing];
    Mr{1}       = [range];
    A_hat{1}    = eye(1);
end