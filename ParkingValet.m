classdef ParkingValet < handle
    properties
        pr; % parameters defined at params.m
        lib; % load 3rdparty functions in Trajectory Planning folder
        vehicleDim;
        costmap;
        vehicleSim;
        motionPlanner;
        behavioralPlanner;
        pathAnalyzer;
        lonController;
    end
    methods
        %% Initialization
        function obj = ParkingValet()
            addpath('./Trajectory Planning')
            obj.pr          = params;
            obj.lib         = demoEssentials;
            obj.vehicleDim  = obj.pr.carDims;
            obj.costmap             = obj.generateParkingSpaceCostmap(); % Generate and Visualize Parking Space with Cameras
            obj.vehicleSim          = HelperVehicleSimulator(obj.costmap, obj.vehicleDim);
            obj.vehicleSim.setVehiclePose(obj.pr.p_0')
            obj.vehicleSim.setVehicleVelocity(0.0)
            obj.showCameras()
            obj.motionPlanner       = pathPlannerRRT(obj.costmap);
            obj.behavioralPlanner   = HelperBehavioralPlanner(obj.pr.routePlan, obj.pr.maxSteeringAngle);
            obj.lonController       = HelperLongitudinalController('SampleTime', obj.pr.sampleTime);
        end
        
        %% Derived from Parking Valet Example
        function simulation(obj)
            currentPose     = obj.vehicleSim.getVehiclePose(); % Input to planner should be a row vector
            currentVel      = obj.vehicleSim.getVehicleVelocity();
            while ~reachedDestination(obj.behavioralPlanner)
                
                % Request next maneuver from behavioral layer
                [nextGoal, plannerConfig, speedConfig] = requestManeuver(obj.behavioralPlanner, currentPose, currentVel);
                
                % Configure the motion planner
                plannerConfig.MinTurningRadius  = obj.pr.MinTurningRadius;
                obj.lib.configurePlanner(obj.motionPlanner, plannerConfig);
                
                % Plan a reference path using RRT* planner to the next goal pose
                refPath = plan(obj.motionPlanner, currentPose, nextGoal);
                
                % Check if the path is valid. If the planner fails to compute a path,
                % or the path is not collision-free because of updates to the map, the
                % system needs to re-plan. This scenario uses a static map, so the path
                % will always be collision-free.
                isReplanNeeded = ~checkPathValidity(refPath, obj.costmap);
                if isReplanNeeded
                    warning('Unable to find a valid path. Attempting to re-plan.')
                    % Request behavioral planner to re-plan
                    replanNeeded(obj.behavioralPlanner);
                    continue;
                end
                
                % Retrieve transition poses and directions from the planned path
                [transitionPoses, directions] = interpolate(refPath);
                
                % Smooth the path
                numSmoothPoses   = round(refPath.Length / obj.pr.approxSeparation);
                [refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
                
                % Generate a velocity profile
                startSpeed      = 0; % in meters/second
                endSpeed        = 0; % in meters/second
                refVelocities   = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, obj.pr.maxSpeed);
                
                % Configure path analyzer
                obj.pathAnalyzer = HelperPathAnalyzer(refPoses, refVelocities, directions, 'Wheelbase', obj.vehicleDim.Wheelbase);
                
                % Reset longitudinal controller
                reset(obj.lonController);
                
                reachGoal = false;
                
                % Execute control loop
                while ~reachGoal
                    % Find the reference pose on the path and the corresponding velocity
                    [refPose, refVel, direction] = obj.pathAnalyzer(currentPose, currentVel);
                    
                    % Update driving direction for the simulator
                    obj.vehicleSim.updateDrivingDirection(direction);
                    
                    % Compute steering command
                    steeringAngle   = lateralControllerStanley(refPose, currentPose, currentVel, ...
                        'Direction', direction, 'Wheelbase', obj.vehicleDim.Wheelbase);
                    
                    % Compute acceleration and deceleration commands
                    obj.lonController.Direction     = direction;
                    [accelCmd, decelCmd]            = obj.lonController(refVel, currentVel);
                    
                    % Simulate the vehicle using the controller outputs
                    drive(obj.vehicleSim, accelCmd, decelCmd, steeringAngle);
                    
                    % Check if the vehicle reaches the goal
                    reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
                    
                    % Wait for fixed-rate execution
                    controlRate = HelperFixedRate(1/obj.pr.sampleTime);
                    waitfor(controlRate);
                    
                    % Get current pose and velocity of the vehicle
                    currentPose  = getVehiclePose(obj.vehicleSim);
                    currentVel   = getVehicleVelocity(obj.vehicleSim);
                end
            end
            % Show vehicle simulation figure
            showFigure(obj.vehicleSim);
        end
        
        %% Parking Space Map: Build up binary array representation for parking space
        function costmap = generateParkingSpaceCostmap(obj) 
            mapLayers   = generateParkingLots(obj.pr.carDims, obj.pr.SpaceDim, obj.pr.numLotsPerRow,...
                                              obj.pr.occupiedLots, obj.pr.lotSize, obj.pr.ratioMeter2Pixel);
            resolution  = obj.pr.SpaceDim(1)/size(mapLayers.StationaryObstacles, 2); % resolution of the occupancy grids in meter
            costmap     = obj.lib.combineMapLayers(mapLayers, resolution);
            ccConfig    = inflationCollisionChecker(obj.vehicleDim, obj.pr.numCircles);
            costmap.CollisionChecker    = ccConfig;
        end
        
        %% Visualize cameras and their FoV
        function showCameras(obj)
            plot(obj.costmap, 'Inflation', 'off'); hold on;
            for i = 1:obj.pr.m
                patch   = obj.get_wedge_patch(i);
                fill(patch(1,:), patch(2,:), 'red', 'FaceAlpha', 0.05, 'EdgeAlpha', 0.0);
                plot(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
                offset  = (-1)^(obj.pr.l_hat(2,i) < obj.pr.SpaceDim(2))*2;
                text(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i)+offset, num2str(i), 'Color', 'Red', 'FontSize', 20)
            end
        end
        
        function patch = get_wedge_patch(obj, i)
            [x,y,theta,FoV,r] = deal(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i), obj.pr.l_hat(3,i), obj.pr.FoV, obj.pr.Measurable_R);
            theta_l     = theta - 0.5*FoV;
            theta_u     = theta + 0.5*FoV;
            thetas      = theta_l:pi/180:theta_u;
            patch       = [x;y];
            
            for t = thetas
                point   = [x+r*cos(t); y+r*sin(t)];
                patch   = [patch, point];
            end
        end
    end
end
