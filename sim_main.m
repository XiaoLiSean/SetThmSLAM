clear all; clc;
close all;

%% Initialize Parking Space and Visualization
cameraType              = 'mono'; % ['mono'/'stereo']
isReconstruction        = true; % reconstruction and plot the defined vehicle state instead of the markers
enableCamUpdate         = true; % enable update camera set/particle
enableCtrlSignal        = false; % enable pass control signal to propagate sets/particles
% =======================================================
enableSetSLAM           = true;
enableRBConstraints     = true; % [true/false] to enable rigid body constraint in set update
% =======================================================
enableFastSLAM          = true;
% =======================================================
PV                      = ParkingValet(cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM, enableRBConstraints, isReconstruction);
PV.simulation();