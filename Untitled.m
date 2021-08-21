clear all; clc;
close all;

%% Initialize Parking Space and Visualization
saveHistoryConcise      = true; % disable save full history of the simulation
usePrevTrajectory       = true; % use previous stored path
isReconstruction        = true; % reconstruction and plot the defined vehicle state instead of the markers
enableCamUpdate         = [false, true]; % enable update camera set/particle
enableCtrlSignal        = [false, true]; % enable pass control signal to propagate sets/particles
% =======================================================
enableSetSLAM           = true;
enableRBConstraints     = true; % [true/false] to enable rigid body constraint in set update
% =======================================================
enableFastSLAM          = true;
% =======================================================
%% Change Parameters
cameraTypes     = ["mono", "stereo"];
parameters      = params;
num             = 9;
e_vas           = deg2rad(linspace(0.1, 10, num));
e_vrs           = linspace(0.01, 0.5, num);
epsilon_Lts     = deg2rad(linspace(0.1, 10, num));
epsilon_Lxys    = linspace(0.01, 0.5, num);
epsilon_Ps      = linspace(0.1, 5, num);
e_mesh          = {e_vas, e_vrs, epsilon_Lts, epsilon_Lxys, epsilon_Ps};                                    
%% Simulation Main
History     = load('Path.mat').Historys;
parameters.plotTime     = inf; % disable the graph update
for cameraType = cameraTypes
for k = 1
    if k == 2 && strcmp(cameraType,'mono') 
        continue
    end
    % =====================================================================
    pr      = {deg2rad(0.1), 0.01, deg2rad(0.1), 0.01, 0.1};
    for i = 1:num
        disp('Data collection '+cameraType+num2str(k)+'_'+num2str(i)+'_'+num2str(e_mesh{k}(i)))
        pr{k}   = e_mesh{k}(i);
        [e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P]    = deal(pr{:});
        %disp([e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P])
        parameters.resetParams(e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P);
        PV                      = ParkingValet(parameters, cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM,...
                                               enableRBConstraints, isReconstruction, enableCtrlSignal, saveHistoryConcise);
        PV.simulateHistory(History);
        saveHistory(PV, cameraType)
    end
end
end

%% Support Function
function saveHistory(PV, cameraType)
    filename    = "graphData/" + cameraType + "_eva_" + num2str(PV.pr.e_va) + "_evr_" + num2str(PV.pr.e_vr) + ...
                    "_ew_" + num2str(PV.pr.e_w(1)) + "_Lt0_" + num2str(PV.pr.epsilon_Lt) + ...
                    "_Lxy0_" + num2str(PV.pr.epsilon_Lxy) + "_P0_" + num2str(PV.pr.epsilon_P) + '.mat';
    Historys    = PV.History;
    save(filename, 'Historys');
end