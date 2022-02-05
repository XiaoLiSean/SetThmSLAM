clear all; clc;
close all;

global trajHis

%% Initialize Parking Space and Visualization
cameraType              = 'stereo'; % ['mono'/'stereo']
knownDataAssociation    = true; % if the measurement to marker associations are known
saveHistory             = false; % if save the simulation history
saveHistoryConcise      = false & saveHistory; % disable save full history of the simulation
usePrevTrajectory       = true; % use previous stored path
isReconstruction        = false; % reconstruction and plot the defined vehicle state instead of the markers
enableCamUpdate         = [false, true]; % enable update camera set/particle
enableCtrlSignal        = [false, true]; % enable pass control signal to propagate sets/particles
% =======================================================
enableSetSLAM           = false;
enableRBConstraints     = false; % [true/false] to enable rigid body constraint in set update
% =======================================================
enableFastSLAM          = false;
% =======================================================
PV                      = ParkingValet(params, cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM, knownDataAssociation, ...
                                        enableRBConstraints, isReconstruction, enableCtrlSignal, saveHistory, saveHistoryConcise);

%% Simulation Main
History     = load('Path.mat').Historys;
PV.simulateHistory(History);
