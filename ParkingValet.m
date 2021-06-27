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
        FastSLAM;
        
        %% Measurement information
        isStereoVision;
        Measurable_R;
        Ma; % Ma{i} angle measurement set for camera i
        Mr; % Mr{i} range measurement set for camera i
        A; % A{i} is association matrix for measurement from camera i
        A_hat; % A_hat{i} is ground true matching solution matrix for measurement from camera i
        
        %% Bounded error for uncertainty sets
        e_va; % angle measurement noise is bounded by e_va
        e_vr; % range measurement noise is bounded by e_vr
        
        %% Nominal states and plot handler for markers and camera
        lxy_hat; % Nominal camera position
        lt_hat; % Nominal camera heading
        p_hat; % Nominal marker position
        p_hat_rel; % Nominal marker position in car frame
        p_car; % Nominal current vehicle states
        
        %% Variables used to reconstruct vehicle states
        isReconstruction; % reconstruction and plot the defined vehicle state instead of the markers
        reconRefIdx; % index of referred marker for vehicle heading reconstruction
        pxy; % nominal states of the center of mass (CoM) of the rigid body skeleton
        pt; % nominal vehicle heading
        
        %% Plot handlers
        h_p_hat; % plot handler for nominal marker position
        h_Lxy; % handler for Camera position uncertainty set
        h_Lt; % handler for Camera heading uncertainty set
        h_P; % handler for marker position uncertainty set
        h_pxy; % handler for nominal states of the center of mass (CoM) of the rigid body skeleton
        h_pt; % handler for nominal vehicle heading
        h_Pxy; % handler for reconstructed uncertainty sets
        h_Pt; % handler for reconstructed uncertainty sets
        
    end
    methods
        %% Initialization
        function obj = ParkingValet(cameraType, enableRigidBodyConstraints, isReconstruction)
            addpath('./util')
            addpath('./set operation')
            addpath('./partical filtering')
            % -------------------------------------------------------------
            obj.pr  = params;
            if strcmp(cameraType,'stereo')
                obj.isStereoVision  = true;
                obj.e_va            = obj.pr.e_va;
                obj.e_vr            = obj.pr.e_vr;
            elseif strcmp(cameraType,'mono')
                obj.isStereoVision  = false;
                obj.e_va            = obj.pr.e_va;
            end
            obj.Measurable_R    = obj.pr.Measurable_R;
            obj.p_hat_rel       = obj.pr.p_hat_rel;
            obj.p_car           = obj.pr.p_0;
            for i = 1:obj.pr.n
                obj.p_hat{i}    = [obj.pr.p_hat(1,i); obj.pr.p_hat(2,i)];
            end
            for i = 1:obj.pr.m
                obj.lxy_hat{i}  = [obj.pr.l_hat(1,i); obj.pr.l_hat(2,i)];
                obj.lt_hat{i}   = obj.pr.l_hat(3,i);
            end
            obj.isReconstruction    = isReconstruction;
            if obj.isReconstruction
                obj.reconRefIdx     = obj.pr.ref_marker;
                obj.updateReconstructedNominalStates();
            end
            % -------------------------------------------------------------
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
            obj.SetSLAM             = SetThmSLAM(obj.pr, obj.isStereoVision, enableRigidBodyConstraints, isReconstruction, obj.p_hat_rel);
            obj.FastSLAM            = FastSLAM(obj.pr, obj.isStereoVision);
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
                    obj.FastSLAM.propagateParticles();
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
                    obj.updateNominalStates(currentPose);
                    if obj.isReconstruction
                        obj.updateReconstructedNominalStates()
                    end
                    obj.updateMeasurements();
                    obj.SetSLAM.getMeasureAndMatching(obj.Ma, obj.Mr, obj.A_hat);
                    obj.FastSLAM.getMeasureAndMatching(obj.Ma, obj.Mr, obj.A_hat);
                    obj.SetSLAM.updateSets();
                    obj.FastSLAM.updateParticles();
                end
                % =====================================================
                % Update Plot and Check if nominal states are in corresponding sets
                if mod(current_time, obj.pr.plotTime) < epsilon
                    obj.updateNominalStates(currentPose);
                    obj.eraseDrawing();
                    obj.drawSets();
                    obj.vehicleSim.updatePlot();
                    obj.check_guaranteed_property()
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

        %% Obtain measurement from the CCTV system
        function updateMeasurements(obj)
            obj.Ma      = {};
            obj.A_hat   = {};
            for i = 1:obj.pr.m
                obj.Ma{i}       = [];
                obj.Mr{i}       = [];
                obj.A_hat{i}    = [];
                for j = 1:obj.pr.n
                    [isMeasurable, alpha, range]    = obj.isMeasurable(obj.p_hat{j}, obj.lxy_hat{i}, obj.lt_hat{i});
                    if isMeasurable
                        idx             = length(obj.Ma{i}) + 1;
                        obj.Ma{i}(idx)  = alpha;
                        if obj.isStereoVision
                            obj.Mr{i}(idx)      = range;
                        end
                        obj.A_hat{i}(idx,:)     = zeros(1, obj.pr.n);
                        obj.A_hat{i}(idx,j)     = 1; % j'th marker associated with idx'th alpha measurement
                    end
                end
            end
        end
        
        function [isMeasurable, alpha, range] = isMeasurable(obj, p, lxy, lt)
            [angle, distance, isMeasurable] = measureModel(p, [lxy; lt], obj.Measurable_R, obj.pr.FoV);
            noise_a     = -obj.e_va + 2*obj.e_va*rand();
            alpha       = angle + noise_a;
            range       = nan;
            if obj.isStereoVision
                noise_r = -obj.e_vr + 2*obj.e_vr*rand();
                range   = distance + noise_r;
            end
        end
        
        %% Update nominal states
        % update marker position p_hat using nominal par states p_car
        function updateNominalStates(obj, p_car)
            obj.p_car   = [p_car(1); p_car(2); deg2rad(p_car(3))];
            for i = 1:obj.pr.n
                x_marker_i      = obj.p_car(1) + obj.p_hat_rel(1,i)*cos(obj.p_car(3)) - obj.p_hat_rel(2,i)*sin(obj.p_car(3));
                y_marker_i      = obj.p_car(2) + obj.p_hat_rel(1,i)*sin(obj.p_car(3)) + obj.p_hat_rel(2,i)*cos(obj.p_car(3));
                obj.p_hat{i}    = [x_marker_i; y_marker_i];
            end
        end
        
        %% update reconstructed states: CoM of rigid body skeleton and vehicle heading
        function updateReconstructedNominalStates(obj)
            obj.pxy     = zeros(2,1);
            for i = 1:obj.pr.n
                obj.pxy     = obj.pxy + obj.p_hat{i}/obj.pr.n;
            end
            delta   = obj.p_hat{obj.reconRefIdx} - obj.pxy;
            obj.pt  = atan2(delta(2), delta(1));
        end
        
        %% function used to check if the nominal state is in the corresponding sets
        function check_guaranteed_property(obj)
            for i = 1:obj.pr.n
                if in(obj.SetSLAM.P{i}, obj.p_hat{i}) == 0
                    error('nominal state outside the set');
                end
            end
            for i = 1:obj.pr.m
                if in(obj.SetSLAM.Lxy{i}, obj.lxy_hat{i}) == 0 ||...
                    (in(obj.SetSLAM.Lt{i}, wrapToPi(obj.lt_hat{i})) == 0 && in(obj.SetSLAM.Lt{i}, wrapTo2Pi(obj.lt_hat{i})) == 0)
                    error('nominal state outside the set');
                end
            end
        end
        
        %% Visualization
        % this function draw the camera at initialization
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
        % visualization: update and erase dynamic plot
        function drawSets(obj)
            if obj.isReconstruction
                obj.h_Pxy   = plot(obj.SetSLAM.Pxy);
                obj.h_pxy   = plot(obj.pxy(1), obj.pxy(2), 'r.', 'MarkerSize', 10);
                r           = norm(obj.pxy-obj.p_hat{obj.reconRefIdx}, 2);
                t1          = obj.SetSLAM.Pt.inf;
                t2          = obj.SetSLAM.Pt.sup;
                x           = [obj.pxy(1)+r*cos(t1), obj.pxy(1), obj.pxy(1)+r*cos(t2)];
                y           = [obj.pxy(2)+r*sin(t1), obj.pxy(2), obj.pxy(2)+r*sin(t2)];
                obj.h_Pt    = plot(x, y, 'b');
                obj.h_pt    = plot([obj.pxy(1), obj.p_hat{obj.reconRefIdx}(1)], [obj.pxy(2), obj.p_hat{obj.reconRefIdx}(2)], 'r--');
            else
                for i = 1:obj.pr.n
                    obj.h_P{i}      = plot(obj.SetSLAM.P{i});
                    obj.h_p_hat{i}  = plot(obj.p_hat{i}(1), obj.p_hat{i}(2), 'r.', 'MarkerSize', 10);   
                end
            end

            for i = 1:obj.pr.m
                obj.h_Lxy{i}    = plot(obj.SetSLAM.Lxy{i});
                t1  = obj.SetSLAM.Lt{i}.inf;
                t2  = obj.SetSLAM.Lt{i}.sup;
                r   = 0.25*obj.Measurable_R; % line length to visualize the heading uncertainty
                x   = [obj.lxy_hat{i}(1)+r*cos(t1), obj.lxy_hat{i}(1), obj.lxy_hat{i}(1)+r*cos(t2)];
                y   = [obj.lxy_hat{i}(2)+r*sin(t1), obj.lxy_hat{i}(2), obj.lxy_hat{i}(2)+r*sin(t2)];
                obj.h_Lt{i}     = plot(x, y, 'b');
            end
        end
        
        function eraseDrawing(obj)
            if isempty(obj.h_Lxy)
                return
            end
            
            if obj.isReconstruction
                delete(obj.h_Pxy)
                delete(obj.h_Pt)
                delete(obj.h_pxy)
                delete(obj.h_pt)
            else
                for i = 1:obj.pr.n
                    delete(obj.h_P{i})
                    delete(obj.h_p_hat{i})
                end
            end
            
            for i = 1:obj.pr.m
                delete(obj.h_Lxy{i})
                delete(obj.h_Lt{i})
            end
        end
    end
end
