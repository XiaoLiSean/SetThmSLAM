clear all; clc;
close all;

%% Initialize Parking Space and Visualization
cameraType              = 'stereo'; % ['mono'/'stereo']
enableRBConstraints     = false; % [true/false] to enable rigid body constraint in set update
isReconstruction        = false; % reconstruction and plot the defined vehicle state instead of the markers
PV                      = ParkingValet(cameraType, enableRBConstraints, isReconstruction);
PV.simulation();
