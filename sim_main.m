clear all; clc;
close all;

%% Initialize Parking Space and Visualization
cameraType              = 'stereo'; % ['mono'/'stereo']
knownDataAssociation    = true; % if the measurement to marker associations are known
saveHistory             = false; % if save the simulation history
saveHistoryConcise      = true & saveHistory; % disable save full history of the simulation
usePrevTrajectory       = true; % use previous stored path
isReconstruction        = false; % reconstruction and plot the defined vehicle state instead of the markers
enableCamUpdate         = [false, true]; % enable update camera set/particle
enableCtrlSignal        = [false, true]; % enable pass control signal to propagate sets/particles
% =======================================================
enableSetSLAM           = true;
enableRBConstraints     = true; % [true/false] to enable rigid body constraint in set update
% =======================================================
enableFastSLAM          = false;
% =======================================================
PV                      = ParkingValet(params, cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM, knownDataAssociation, ...
                                        enableRBConstraints, isReconstruction, enableCtrlSignal, saveHistory, saveHistoryConcise);

%% Simulation Main
if ~usePrevTrajectory
    PV.simulation();
else
    History     = load('Path.mat').Historys;
    PV.simulateHistory(History);
end
saveSimHistory(PV, cameraType, usePrevTrajectory)

%% Support Function
function saveSimHistory(PV, cameraType, usePrevTrajectory)
    if usePrevTrajectory
        filename    = "graphData/snapshot_" + cameraType + "_eva_" + num2str(PV.pr.e_va) + "_evr_" + num2str(PV.pr.e_vr) + ...
                        "_ew_" + num2str(PV.pr.e_w(1)) + "_Lt0_" + num2str(PV.pr.epsilon_Lt) + ...
                        "_Lxy0_" + num2str(PV.pr.epsilon_Lxy) + "_P0_" + num2str(PV.pr.epsilon_P) + '.mat';
    else
        filename    = 'Path.mat';
    end
    Historys                = PV.History;
    save(filename, 'Historys');
end