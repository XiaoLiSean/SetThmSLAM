clear all; clc;
close all;

%% Initialize Parking Space and Visualization
cameraType              = 'stereo'; % ['mono'/'stereo']
enableSetSLAM           = true;
enableFastSLAM          = false;
enableRBConstraints     = false; % [true/false] to enable rigid body constraint in set update
isReconstruction        = false; % reconstruction and plot the defined vehicle state instead of the markers
PV                      = ParkingValet(cameraType, enableFastSLAM, enableSetSLAM, enableRBConstraints, isReconstruction);
PV.simulation();
