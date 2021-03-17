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
            obj.costmap             = generateParkingSpaceMap(1); % Generate and Visualize Parking Space with Cameras
            obj.vehicleSim          = HelperVehicleSimulator(obj.costmap, obj.vehicleDim);
            obj.motionPlanner       = pathPlannerRRT(obj.costmap, 'MinIterations', 1000,...
                                      'ConnectionDistance', 10, 'MinTurningRadius', obj.pr.MinTurningRadius);
            obj.behavioralPlanner   = HelperBehavioralPlanner(obj.pr.routePlan, obj.pr.maxSteeringAngle);
            obj.lonController       = HelperLongitudinalController('SampleTime', obj.pr.sampleTime);
        end
        
        %% Derived from Parking Valet Example
        function simulation(obj)
            currentPose     = obj.pr.p_0'; % Input to planner should be a row vector
            currentVel      = 0;
            while ~reachedDestination(obj.behavioralPlanner)
                
                % Request next maneuver from behavioral layer
                [nextGoal, plannerConfig, speedConfig] = requestManeuver(obj.behavioralPlanner, currentPose, currentVel);
                
                % Configure the motion planner
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
    end
end
