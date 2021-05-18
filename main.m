clear all; clc;
close all;

%% Initialize Parking Space and Visualization
enableRBConstraints     = true; % [true/false]
cameraType              = 'mono'; % ['mono'/'stereo']
PV                      = ParkingValet(cameraType, enableRBConstraints);
PV.simulation();
