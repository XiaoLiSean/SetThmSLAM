clear all; clc;
close all;

%% Set up Parking Space and Visualize
costmap     = generateParkingSpaceMap();
plot(costmap, 'Inflation', 'off'); hold on;