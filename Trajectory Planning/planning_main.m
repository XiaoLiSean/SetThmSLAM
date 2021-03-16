clear all; clc;
close all;

lib         = demoEssentials; % load 3rdparty functions 

%% Initialize car dimension (in meter) which will be scale up by 10 times
carLength   = 0.255; % real car length is 0.255 meters
carWidth    = 0.230; % real car width is 0.230 meters
carHeight   = 0.14; % real car height is 0.14 meters
carDims     = vehicleDimensions(carLength,carWidth,carHeight,...
                                'FrontOverhang',0.04,'RearOverhang',0.04);
                            
%% Initialize map configurations and pixel array
mapDims             = 0.3*[12, 12]; % [size in x axis, size in y axis] in meters
occupiedLots        = [0, 1, 0, 0;
                       1, 0, 0, 0]; % Occupancy array of parking lots
numLotsPerRow       = size(occupiedLots, 2); % Two rows of parking lots in the middle of the map and numLotsPerRow for each row
lotSize             = [0.4, 0.4]; % Pazrking lot is x*y m^2
ratioMeter2Pixel    = 80; % Increase for higher resolution
mapLayers   = generateParkingLots(carDims, mapDims, numLotsPerRow, occupiedLots, lotSize, ratioMeter2Pixel);
resolution  = mapDims(1)/size(mapLayers.StationaryObstacles, 2); % resolution of the occupancy grids in meter
costmap     = lib.combineMapLayers(mapLayers, resolution);

figure(1)
costmap.CollisionChecker.VehicleDimensions  = carDims;
costmap.CollisionChecker.NumCircles         = 4;
plot(costmap); hold on;

%% Set up vehical states, model and dynamics parameters
maxSteeringAngle                            = rad2deg(0.4); % in degrees
MinTurningRadius                            = 0.4; % in meters
MaxWayPointDistance                         = 0.2; % in meters
currentPose                                 = [0.5 0.25 0]; % [x, y, theta]
helperPlotVehicle(currentPose, carDims, 'DisplayName', 'Current Pose'); legend;

%% Trajectory and velocity planning
motionPlanner   = pathPlannerRRT(costmap, 'MinIterations', 1000,...
                                 'MinTurningRadius', MinTurningRadius);
goalPose        = [2.0, 1.4, 90];
refPath         = plan(motionPlanner, currentPose, goalPose);
[transitionPoses, directions]   = interpolate(refPath);
numSmoothPoses  = round(refPath.Length / MaxWayPointDistance);
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
currentVel  = 0;
endSpeed    = 0;
maxVelocity = 0.27;
refVelocities   = helperGenerateVelocityProfile(directions, cumLengths, curvatures, currentVel, endSpeed, maxVelocity);

figure(2)
plot(costmap); hold on;
plot(motionPlanner);
plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, 'DisplayName', 'Smoothed Path');
hold off;