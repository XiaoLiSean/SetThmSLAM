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
        SetSLAM;
    end
    methods
        %% Initialization
        function obj = ParkingValet(cameraType, enableRigidBodyConstraints)
            addpath('./util')
            addpath('./set operation')
            obj.pr          = params;
            obj.lib         = demoEssentials;
            obj.vehicleDim  = obj.pr.carDims;
            obj.costmap             = obj.generateParkingSpaceCostmap(); % Generate and Visualize Parking Space with Cameras
            % Note: HelperVehicleSimulator will initialize two seperate
            % threads every 0.01 sec (defined as obj.Step): one in 
            % HelperKinematicVehicle.updateKinematics to update the vehicle
            % states; another to plot the new updated vehicle
            obj.vehicleSim          = HelperVehicleSimulator(obj.costmap, obj.vehicleDim);
            obj.vehicleSim.setVehiclePose(obj.pr.p_0')
            obj.vehicleSim.setVehicleVelocity(0.0)
            obj.showCameras()
            obj.motionPlanner       = pathPlannerRRT(obj.costmap);
            obj.behavioralPlanner   = HelperBehavioralPlanner(obj.pr.routePlan, obj.pr.maxSteeringAngle);
            obj.lonController       = HelperLongitudinalController('SampleTime', obj.pr.sampleTime);
            obj.SetSLAM             = SetThmSLAM(obj.pr, cameraType, enableRigidBodyConstraints);
        end
        
        %% Derived from Parking Valet Example
        function simulation(obj)
            current_time    = 0;
            reachGoal       = true; % for initial planning
            epsilon         = 0.01*obj.pr.simLoopDt; % take care of numerical error in computation
            % Simulation loop for every obj.pr.simLoopDt
            while true
                % =====================================================
                % Plan new path if sub-goal is reached
                currentPose = obj.vehicleSim.getVehiclePose();
                currentVel  = obj.vehicleSim.getVehicleVelocity();
                % Plan for new sub-goal if the current goal is reached
                while reachGoal
                    % Break the simulation loop if reached the final destination
                    if obj.behavioralPlanner.reachedDestination()
                        return;
                    end
                    [nextGoal, speedConfig, isReplanNeeded] = obj.pathPlanner(currentPose, currentVel);
                    if isReplanNeeded
                        continue;
                    else
                        break;
                    end
                end             
                % =====================================================
                % Control Update of Vehicle states and sets
                if mod(current_time, obj.pr.propTime) < epsilon
                    obj.vehicleSim.updateKinematics(obj.pr.propTime);
                    currentPose = obj.vehicleSim.getVehiclePose();
                    currentVel  = obj.vehicleSim.getVehicleVelocity();
                    obj.SetSLAM.propagateSets();
                end
                % =====================================================
                % Update Control Signal
                if mod(current_time, obj.pr.sampleTime) < epsilon
                    % Calculate control signal using stanley(steeringAngle) + pi(accelCmd, decelCmd) controller
                    [accelCmd, decelCmd, steeringAngle, direction] = obj.Controller(currentPose, currentVel);
                    obj.vehicleSim.drive(accelCmd, decelCmd, steeringAngle); 
                end
                % =====================================================
                % Measurement Update Sets
                if mod(current_time, obj.pr.updateTime) < epsilon
                    obj.SetSLAM.updateNominalStates(currentPose);
                    obj.SetSLAM.updateMeasurements();
                    obj.SetSLAM.matching();
                    obj.SetSLAM.updateSets();
                end
                % =====================================================
                % Update Plot and Check if nominal states are in corresponding sets
                if mod(current_time, obj.pr.plotTime) < epsilon
                    obj.SetSLAM.updateNominalStates(currentPose);
                    obj.SetSLAM.eraseDrawing();
                    obj.SetSLAM.drawSets();
                    obj.SetSLAM.check_guaranteed_property()
                    obj.vehicleSim.updatePlot();
                end
                % =====================================================
                % Check if the sub-goal is reached
                current_time    = current_time + obj.pr.simLoopDt;
                reachGoal       = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
                % =====================================================
            end
            % Show vehicle simulation figure
            obj.vehicleSim.showFigure();
        end
        
        function [accelCmd, decelCmd, steeringAngle, direction] = Controller(obj, currentPose, currentVel)
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
        end
        
        function [nextGoal, speedConfig, isReplanNeeded] = pathPlanner(obj, currentPose, currentVel)
            % Request next maneuver from behavioral layer
            [nextGoal, plannerConfig, speedConfig] = obj.behavioralPlanner.requestManeuver(currentPose, currentVel);
            
            % Configure the motion planner
            plannerConfig.MinTurningRadius  = obj.pr.MinTurningRadius;
            obj.lib.configurePlanner(obj.motionPlanner, plannerConfig);
            
            % Plan a reference path using RRT* planner to the next goal pose
            refPath = obj.motionPlanner.plan(currentPose, nextGoal);
            
            % Check if the path is valid. If the planner fails to compute a path,
            % or the path is not collision-free because of updates to the map, the
            % system needs to re-plan. This scenario uses a static map, so the path
            % will always be collision-free.
            isReplanNeeded = ~checkPathValidity(refPath, obj.costmap);
            if isReplanNeeded
                warning('Unable to find a valid path. Attempting to re-plan.')
                % Request behavioral planner to re-plan
                obj.behavioralPlanner.replanNeeded();
                return
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
            obj.lonController.reset();
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
            set(gcf,'color','w');
            plot(obj.costmap, 'Inflation', 'off'); hold on;
            xlim([obj.pr.Omega_L.inf(1) obj.pr.Omega_L.sup(1)]);
            ylim([obj.pr.Omega_L.inf(2) obj.pr.Omega_L.sup(2)]);
            for i = 1:obj.pr.m
                patch   = obj.get_wedge_patch(i);
                fill(patch(1,:), patch(2,:), 'red', 'FaceAlpha', 0.02, 'EdgeAlpha', 0.05);
                plot(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
                offset  = (-1)^(obj.pr.l_hat(2,i) < obj.pr.SpaceDim(2))*2;
                text(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i)+offset, num2str(i), 'Color', 'Red', 'FontSize', 20)
                plot([obj.pr.l_hat(1,i), obj.pr.l_hat(1,i)+0.25*obj.pr.Measurable_R*cos(obj.pr.l_hat(3,i))],...
                     [obj.pr.l_hat(2,i), obj.pr.l_hat(2,i)+0.25*obj.pr.Measurable_R*sin(obj.pr.l_hat(3,i))], 'r--');
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
