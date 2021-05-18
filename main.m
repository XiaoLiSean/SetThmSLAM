clear all; clc;
close all;

%% Initialize Parking Space and Visualization
enableRBConstraints     = false;
PV                      = ParkingValet('stereo', enableRBConstraints);
PV.simulation();
