classdef sensorCalibrationValet < matlab.mixin.Copyable
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
        enableFastSLAM;
        enableSetSLAM;
        enableCtrlSignalProp;
        knownDataAssociation;
        nominalVehicleBody;
        
        %% Plot snapshot
        snapshot_num;
        snapshot_i;
        colors;
        
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
        markerKinematics; % redundant kinematics update for each markers 
        
        %% Variables used to reconstruct vehicle states
        isReconstruction; % reconstruction and plot the defined vehicle state instead of the markers
        reconRefIdx; % index of referred marker for vehicle heading reconstruction
        pxy; % nominal position of the vehicle
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
        h_P_ell; % handler for marker ellipse
        h_Lxy_ell; % handler for Camera position ellipse
        h_Lt_ell; % handler for Camera heading interval
        h_lxy_particle; % camera particles
        h_p_particle; % marker particles
        
        %% Write Plot and Path History to File
        History;
        saveHistory;
        saveHistoryConcise;
        isSetSLAMGuaranteed;
        isFastSLAMGuaranteed;
    end
    methods
        %% Initialization
        function obj = sensorCalibrationValet(parameters, cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM, knownDataAssociation,...
                                        enableRigidBodyConstraints, isReconstruction, enableCtrlSignal,...
                                        saveHistory, saveHistoryConcise)
            addpath('./util')
            addpath('./set operation')
            addpath('./filtering')
            obj.pr  = parameters;
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
            % -------------------------------------------------------------
            obj.History             = {};
            obj.saveHistory         = saveHistory;
            obj.saveHistoryConcise  = saveHistoryConcise;
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
                obj.markerKinematics{i}     = markerKinematics(obj.pr.Wheelbase, obj.p_hat_rel(:,i));
            end
            for i = 1:obj.pr.m
                obj.lxy_hat{i}  = [obj.pr.l_hat(1,i); obj.pr.l_hat(2,i)];
                obj.lt_hat{i}   = obj.pr.l_hat(3,i);
            end
            obj.isReconstruction    = isReconstruction;
            if obj.isReconstruction
                obj.updateReconstructedNominalStates();
            end
            % -------------------------------------------------------------
            obj.enableSetSLAM           = enableSetSLAM;
            obj.enableFastSLAM          = enableFastSLAM;
            obj.enableCtrlSignalProp    = enableCtrlSignal;
            obj.knownDataAssociation    = knownDataAssociation;
            obj.nominalVehicleBody      = mptPolytope(obj.pr.p_hat');
            if enableSetSLAM
                obj.SetSLAM             = SetThmSLAM(obj.pr,  obj.markerKinematics, obj.isStereoVision, obj.knownDataAssociation, enableCamUpdate(1), enableRigidBodyConstraints, isReconstruction, obj.p_hat_rel);
            end
            if enableFastSLAM
                obj.FastSLAM            = FastSLAM(obj.pr, obj.markerKinematics, obj.isStereoVision, obj.knownDataAssociation, enableCamUpdate(2), isReconstruction);
            end
            % -------------------------------------------------------------
            obj.snapshot_num    = 4;
            obj.snapshot_i      = 0;
            obj.colors          = [0.5 0.5 1;1 0 1;0 1 1;0 0 1];
        end
        
        %% Simulation functions
        function simulation(obj)
            current_time    = 0;
            time_step       = 1;
            reachGoal       = true; % for initial planning
            epsilon         = 0.9*obj.pr.simLoopDt; % take care of numerical error in computation
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
                        % Show vehicle simulation figure and save history
                        obj.vehicleSim.showFigure();
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
                propRes     = mod(current_time, obj.pr.propTime);
                if (propRes < epsilon || obj.pr.propTime - propRes < epsilon) && current_time ~= 0
                    obj.vehicleSim.updateKinematics(obj.pr.propTime);
                    obj.updateNominalStates(obj.vehicleSim.getVehiclePose());
                    if obj.enableSetSLAM  
                        if obj.enableCtrlSignalProp(1)
                            steeringCtrl        = wrapToPi(deg2rad(obj.vehicleSim.Vehicle.SteeringAngle));
                            velocityCtrl        = obj.vehicleSim.Vehicle.Velocity;
                            steeringInterval    = interval(steeringCtrl-obj.pr.e_steering, steeringCtrl+obj.pr.e_steering);
                            velocityInterval    = interval(velocityCtrl-obj.pr.e_velocity, velocityCtrl+obj.pr.e_velocity);
                            obj.SetSLAM.propagateSetsWithCtrl(steeringInterval, velocityInterval, obj.pr.propTime)
                        else
                            obj.SetSLAM.propagateSets();
                        end
                    end
                    if obj.enableFastSLAM
                        if obj.enableCtrlSignalProp(2)
                            obj.FastSLAM.propagateParticlesWithCtrl(steeringCtrl, velocityCtrl, obj.pr.propTime)
                        else
                            obj.FastSLAM.propagateParticles();
                        end
                    end
                end
                % =====================================================
                % Update Control Signal
                sampleRes   = mod(current_time, obj.pr.sampleTime);
                if sampleRes < epsilon || obj.pr.sampleTime - sampleRes < epsilon
                    % Calculate control signal using stanley(steeringAngle) + pi(accelCmd, decelCmd) controller
                    [accelCmd, decelCmd, steeringAngle, direction]  = obj.Controller(currentPose, currentVel);
                    obj.vehicleSim.drive(accelCmd, decelCmd, steeringAngle); 
                end
                % =====================================================
                % Measurement Update Sets
                updateRes   = mod(current_time, obj.pr.updateTime);
                if (updateRes < epsilon || obj.pr.updateTime - updateRes < epsilon) && current_time ~= 0
                    if obj.isReconstruction
                        obj.updateReconstructedNominalStates()
                    end
                    obj.updateMeasurements();
                    tic
                    if obj.enableSetSLAM
                        obj.SetSLAM.getMeasureAndMatching(obj.Ma, obj.Mr, obj.A_hat);
                        obj.SetSLAM.updateSets();
                    end
                    if obj.enableFastSLAM
                        obj.FastSLAM.getMeasureAndMatching(obj.Ma, obj.Mr, obj.A_hat);
                        obj.FastSLAM.updateParticles();
                    end
                    toc
                end
                % =====================================================
                % Update Plot and Check if nominal states are in corresponding sets
                plotRes     = mod(current_time, obj.pr.plotTime);
                if plotRes < epsilon || obj.pr.plotTime - plotRes < epsilon
                    obj.eraseDrawing();
                    obj.drawAll();
                    obj.vehicleSim.updatePlot();
                end
                obj.check_guaranteed_property();
                % =====================================================
                % Check if the sub-goal is reached and save history
                obj.updateHistory(time_step);
                current_time    = current_time + obj.pr.simLoopDt;
                time_step       = time_step + 1;
                reachGoal       = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
                % =====================================================
            end
        end
        
        function simulateHistory(obj, prevHistory)
            % Draw Trajectory
            trajectory      = zeros(length(prevHistory), 2);
            for i = 1:length(prevHistory)
                state               = prevHistory{i}.p_car;
                trajectory(i,:)     = [state(1), state(2)];
            end            
            plot(trajectory(:,1), trajectory(:,2), 'LineWidth', 3)
            current_time    = 0;
            time_step       = 1;
            epsilon         = 0.9*obj.pr.simLoopDt; % take care of numerical error in computation
            % Simulation loop for every obj.pr.simLoopDt
            while true
                % =====================================================
                % Plan new path if sub-goal is reached
                if time_step > length(prevHistory)
                    return
                end
                currentPose = prevHistory{time_step}.p_car;
                steeringCtrl= prevHistory{time_step}.steeringCtrl;
                velocityCtrl= prevHistory{time_step}.velocityCtrl;
                % =====================================================
                % Control Update of Vehicle states and sets
                propRes     = mod(current_time, obj.pr.propTime);
                if (propRes < epsilon || obj.pr.propTime - propRes < epsilon) && current_time ~= 0
                    pose_car    = [currentPose(1), currentPose(2), rad2deg(currentPose(3))];                    
                    obj.vehicleSim.setVehiclePose(pose_car);
                    obj.updateNominalStates(pose_car);
                    if obj.enableSetSLAM  
                        if obj.enableCtrlSignalProp(1)
                            steeringInterval    = interval(steeringCtrl-obj.pr.e_steering, steeringCtrl+obj.pr.e_steering);
                            velocityInterval    = interval(velocityCtrl-obj.pr.e_velocity, velocityCtrl+obj.pr.e_velocity);
                            obj.SetSLAM.propagateSetsWithCtrl(steeringInterval, velocityInterval, obj.pr.propTime)
                        else
                            obj.SetSLAM.propagateSets();
                        end
                    end
                    if obj.enableFastSLAM
                        if obj.enableCtrlSignalProp(2)
                            obj.FastSLAM.propagateParticlesWithCtrl(steeringCtrl, velocityCtrl, obj.pr.propTime)
                        else
                            obj.FastSLAM.propagateParticles();
                        end
                    end
                end
                % =====================================================
                % Measurement Update Sets
                updateRes   = mod(current_time, obj.pr.updateTime);
                if (updateRes < epsilon || obj.pr.updateTime - updateRes < epsilon) && current_time ~= 0
                    if obj.isReconstruction
                        obj.updateReconstructedNominalStates()
                    end
                    obj.updateMeasurements();
                    %tic
                    if obj.enableSetSLAM
                        obj.SetSLAM.getMeasureAndMatching(obj.Ma, obj.Mr, obj.A_hat);
                        obj.SetSLAM.updateSets();
                    end
                    if obj.enableFastSLAM
                        obj.FastSLAM.getMeasureAndMatching(obj.Ma, obj.Mr, obj.A_hat);
                        obj.FastSLAM.updateParticles();
                    end
                    %toc
                end
                % =====================================================
                % Update Plot and Check if nominal states are in corresponding sets
                plotRes     = mod(current_time, obj.pr.plotTime);
                if plotRes < epsilon || obj.pr.plotTime - plotRes < epsilon 
                    obj.eraseDrawing();
                    obj.drawAll();
                    obj.vehicleSim.updatePlot();
                end
                obj.check_guaranteed_property()
                % =====================================================
                % Check if the sub-goal is reached and save history
                if obj.saveHistory
                    if obj.saveHistoryConcise
                        obj.updateHistoryConcise(time_step);
                    else
                        obj.updateHistory(time_step);
                    end
                end
                current_time    = current_time + obj.pr.simLoopDt;
                time_step       = time_step + 1;
                % =====================================================
            end
        end
        
        %% Function Derived from Parking Valet Example
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
        function deltaXY = updateNominalStates(obj, p_car)
            obj.p_car   = [p_car(1); p_car(2); deg2rad(p_car(3))];
            deltaXY     = cell(1, obj.pr.n);
            vertices    = [];
            for i = 1:obj.pr.n
                x_marker_i      = obj.p_car(1) + obj.p_hat_rel(1,i)*cos(obj.p_car(3)) - obj.p_hat_rel(2,i)*sin(obj.p_car(3));
                y_marker_i      = obj.p_car(2) + obj.p_hat_rel(1,i)*sin(obj.p_car(3)) + obj.p_hat_rel(2,i)*cos(obj.p_car(3));
                deltaXY{i}      = [x_marker_i; y_marker_i] - obj.p_hat{i};
                obj.p_hat{i}    = [x_marker_i; y_marker_i];
                vertices        = [vertices; x_marker_i, y_marker_i];
            end
            obj.nominalVehicleBody  = mptPolytope(vertices);
        end
        
        %% update reconstructed states: CoM of rigid body skeleton and vehicle heading
        function updateReconstructedNominalStates(obj)
            state   = obj.vehicleSim.getVehiclePose();
            obj.pxy = state(1:2);
            obj.pt  = deg2rad(state(3));
        end
        
        %% function used to check if the nominal state is in the corresponding sets
        function check_guaranteed_property(obj)
            if obj.enableSetSLAM
                obj.isSetSLAMGuaranteed     = true;
                for i = 1:obj.pr.n
                    if in(obj.SetSLAM.P{i}, obj.p_hat{i}) == 0
                        obj.isSetSLAMGuaranteed     = false;
                        error('nominal state outside the set');
                    end
                end
                for i = 1:obj.pr.m
                    if in(obj.SetSLAM.Lxy{i}, obj.lxy_hat{i}) == 0 ||...
                        (in(obj.SetSLAM.Lt{i}, wrapToPi(obj.lt_hat{i})) == 0 && in(obj.SetSLAM.Lt{i}, wrapTo2Pi(obj.lt_hat{i})) == 0)
                        obj.isSetSLAMGuaranteed     = false;
                        error('nominal state outside the set');
                    end
                end
            end
            if obj.enableFastSLAM && obj.isReconstruction     
                obj.isFastSLAMGuaranteed 	= true;           
                for i = 1:obj.pr.n
                    if in(obj.FastSLAM.Pxy, obj.p_hat{i}) == 0
                        % disp('FastSLAM: nominal marker state outside the set');     
                        obj.isFastSLAMGuaranteed    = false;
                    end
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
        function drawAll(obj)
            obj.snapshot_i  = obj.snapshot_i + 1;
            if obj.snapshot_i > obj.snapshot_num
                return
            end
            if obj.isReconstruction
                r           = obj.pr.carLength;
                obj.h_pxy   = plot(obj.pxy(1), obj.pxy(2), 'r.', 'MarkerSize', 10);
                obj.h_pt    = plot([obj.pxy(1), obj.pxy(1)+r*cos(obj.pt)], [obj.pxy(2), obj.pxy(2)+r*sin(obj.pt)], 'r--');
                if obj.enableSetSLAM
                    obj.h_Pxy{1}    = plot(obj.SetSLAM.Pxy, [1,2], 'b', 'Filled', true, 'FaceColor', obj.colors(obj.snapshot_i,:),...
                        'FaceAlpha', 0.25, 'EdgeColor', obj.colors(obj.snapshot_i,:));
                    t1              = obj.SetSLAM.Pt.inf;
                    t2              = obj.SetSLAM.Pt.sup;
                    x               = [obj.pxy(1)+r*cos(t1), obj.pxy(1), obj.pxy(1)+r*cos(t2)];
                    y               = [obj.pxy(2)+r*sin(t1), obj.pxy(2), obj.pxy(2)+r*sin(t2)];
                    obj.h_Pt{1}     = plot(x, y, 'b');
                end
                if obj.enableFastSLAM
                    obj.h_Pxy{2}    = plot(obj.FastSLAM.Pxy, [1, 2], 'g');
                    t1              = obj.FastSLAM.Pt.inf;
                    t2              = obj.FastSLAM.Pt.sup;
                    x               = [obj.pxy(1)+r*cos(t1), obj.pxy(1), obj.pxy(1)+r*cos(t2)];
                    y               = [obj.pxy(2)+r*sin(t1), obj.pxy(2), obj.pxy(2)+r*sin(t2)];
                    obj.h_Pt{2}     = plot(x, y, 'g');
                end
            else
                for i = 1:obj.pr.n
                    if obj.enableSetSLAM
                        obj.h_P{i}      = plot(obj.SetSLAM.P{i});
                    end
                    if obj.enableFastSLAM
                        obj.h_P_ell{i}  = draw_ellipse(obj.FastSLAM.mu.marker(:,i), obj.FastSLAM.Sigma.marker{1,i}, 9);
                        pos             = zeros(2, obj.FastSLAM.s);
                        for k = 1:obj.FastSLAM.s
                            pos(:,k)    = obj.FastSLAM.particles{k}.Marker{i}.state;
                        end
                        obj.h_p_particle{i}     = scatter(pos(1,:), pos(2,:), 1,...
                            'MarkerEdgeColor',[0 .5 .5], 'MarkerFaceColor',[0.5,0.5,0.5], 'MarkerFaceAlpha', 0.1);
                    end
                    obj.h_p_hat{i}  = plot(obj.p_hat{i}(1), obj.p_hat{i}(2), 'r.', 'MarkerSize', 15);   
                end
            end

            for i = 1:obj.pr.m
                if obj.enableSetSLAM
                    obj.h_Lxy{i}    = plot(obj.SetSLAM.Lxy{i}, [1,2], 'b', 'Filled', true, 'FaceColor', obj.colors(obj.snapshot_i,:),...
                        'FaceAlpha', 0.25, 'EdgeColor', obj.colors(obj.snapshot_i,:));
                    t1  = obj.SetSLAM.Lt{i}.inf;
                    t2  = obj.SetSLAM.Lt{i}.sup;
                    r   = 0.25*obj.Measurable_R; % line length to visualize the heading uncertainty
                    x   = [obj.lxy_hat{i}(1)+r*cos(t1), obj.lxy_hat{i}(1), obj.lxy_hat{i}(1)+r*cos(t2)];
                    y   = [obj.lxy_hat{i}(2)+r*sin(t1), obj.lxy_hat{i}(2), obj.lxy_hat{i}(2)+r*sin(t2)];
                    obj.h_Lt{i}     = plot(x, y, 'b');
                end
                if obj.enableFastSLAM
                    obj.h_Lxy_ell{i}    = draw_ellipse(obj.FastSLAM.mu.camera(1:2,i), obj.FastSLAM.Sigma.camera{1,i}(1:2,1:2), 9);
                    t   = obj.FastSLAM.mu.camera(3,i);
                    r   = 0.25*obj.Measurable_R; % line length to visualize the heading uncertainty
                    lxy = obj.FastSLAM.mu.camera(1:2,i);
                    x   = [lxy(1)+r*cos(t), lxy(1)];
                    y   = [lxy(2)+r*sin(t), lxy(2)];
                    obj.h_Lt_ell{i}     = plot(x, y, 'g');
                    pos = zeros(2, obj.FastSLAM.s);
                    for k = 1:obj.FastSLAM.s
                        pos(:,k)    = obj.FastSLAM.particles{k}.EKFCamera{i}.state(1:2);
                    end
                    obj.h_lxy_particle{i}   = scatter(pos(1,:), pos(2,:), 1,...
                        'MarkerEdgeColor',[0 .5 .5], 'MarkerFaceColor',[0.5,0.5,0.5], 'MarkerFaceAlpha', 0.1);
                end
            end

            for i = 1:obj.pr.m
                plot(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
                offset  = (-1)^(obj.pr.l_hat(2,i) < obj.pr.SpaceDim(2))*2;
                text(obj.pr.l_hat(1,i), obj.pr.l_hat(2,i)+offset, num2str(i), 'Color', 'Red', 'FontSize', 20)
                plot([obj.pr.l_hat(1,i), obj.pr.l_hat(1,i)+0.25*obj.pr.Measurable_R*cos(obj.pr.l_hat(3,i))],...
                     [obj.pr.l_hat(2,i), obj.pr.l_hat(2,i)+0.25*obj.pr.Measurable_R*sin(obj.pr.l_hat(3,i))], 'r--');
            end
        end
        
        function eraseDrawing(obj)
            if isempty(obj.h_p_hat) && isempty(obj.h_Pxy)
                return
            end            
            
            if obj.isReconstruction
                if obj.enableSetSLAM
                    %delete(obj.h_Pxy{1})
                    delete(obj.h_Pt{1})
                end
                if obj.enableFastSLAM
                    delete(obj.h_Pxy{2})
                    delete(obj.h_Pt{2})
                end                
                delete(obj.h_pxy)
                delete(obj.h_pt)
            else
                for i = 1:obj.pr.n
                    if obj.enableSetSLAM
                        delete(obj.h_P{i})
                    end
                    if obj.enableFastSLAM
                        delete(obj.h_P_ell{i})
                        delete(obj.h_p_particle{i})
                    end
                    delete(obj.h_p_hat{i})
                end
            end
            
%             for i = 1:obj.pr.m
%                 if obj.enableSetSLAM
%                     delete(obj.h_Lxy{i})
%                     delete(obj.h_Lt{i})
%                 end
%                 if obj.enableFastSLAM
%                     delete(obj.h_Lxy_ell{i})
%                     delete(obj.h_Lt_ell{i})
%                     delete(obj.h_lxy_particle{i})
%                 end
%             end
        end
        
        %% Function used to save history of plot and path
        function updateHistory(obj, time_step)
            if time_step == 1
                template.pr         = copy(obj.pr);
                template.costmap    = obj.costmap;
            end
            template.p_hat  = obj.p_hat;
            template.p_car  = obj.p_car;
            template.pxy    = obj.pxy;
            template.pt     = obj.pt;
            template.steeringCtrl           = wrapToPi(deg2rad(obj.vehicleSim.Vehicle.SteeringAngle));
            template.velocityCtrl           = obj.vehicleSim.Vehicle.Velocity;
            template.nominalVehicleBody     = obj.nominalVehicleBody;
            if obj.enableSetSLAM
                template.SetSLAM.Lxy    = obj.SetSLAM.Lxy;
                template.SetSLAM.Lt     = obj.SetSLAM.Lt;
                template.SetSLAM.P      = obj.SetSLAM.P;
                template.SetSLAM.Pxy    = obj.SetSLAM.Pxy;
                template.SetSLAM.Pt     = obj.SetSLAM.Pt;
                template.SetSLAM.isIn   = obj.isFastSLAMGuaranteed;
                template.SetSLAM.PxyOr  = volume(and(obj.SetSLAM.Pxy, obj.nominalVehicleBody));
            end
            if obj.enableFastSLAM
                template.FastSLAM.mu    = obj.FastSLAM.mu;
                template.FastSLAM.Sigma = obj.FastSLAM.Sigma;
                template.FastSLAM.Pxy   = obj.FastSLAM.Pxy;
                template.FastSLAM.Pt    = obj.FastSLAM.Pt;
                template.FastSLAM.isIn  = obj.isFastSLAMGuaranteed;
                template.FastSLAM.PxyOr = volume(and(obj.FastSLAM.Pxy, obj.nominalVehicleBody));
            end
            obj.History{time_step}  = template;
        end
        
        
        function updateHistoryConcise(obj, time_step)            
            if time_step == 1
                template.pr         = copy(obj.pr);
            end
            template.theta_car      = obj.p_car(3); 
            if obj.enableSetSLAM
                template.SetSLAM.Pxy    = volume(obj.SetSLAM.Pxy);
                template.SetSLAM.Pt     = volume(obj.SetSLAM.Pt);
                template.SetSLAM.isIn   = obj.isSetSLAMGuaranteed;
                template.SetSLAM.PxyOr  = volume(and(obj.SetSLAM.Pxy, obj.nominalVehicleBody));
                template.SetSLAM.P      = zeros(4,1);
                for i = 1:obj.pr.n
                    template.SetSLAM.P(i,1)     = volume(obj.SetSLAM.P{i});
                end
                template.SetSLAM.PtInf  = obj.SetSLAM.Pt.inf;
                template.SetSLAM.PtSup  = obj.SetSLAM.Pt.sup;
            end
            if obj.enableFastSLAM
                template.FastSLAM.Pxy   = volume(obj.FastSLAM.Pxy);
                template.FastSLAM.Pt    = volume(obj.FastSLAM.Pt);
                template.FastSLAM.isIn  = obj.isFastSLAMGuaranteed;
                template.FastSLAM.PxyOr = volume(and(obj.FastSLAM.Pxy, obj.nominalVehicleBody));
                template.FastSLAM.P     = zeros(4,1);
                for i = 1:obj.pr.n
                    template.FastSLAM.P(i,1)    = obj.FastSLAM.VP{i};
                end
                template.FastSLAM.PtInf = obj.FastSLAM.Pt.inf;
                template.FastSLAM.PtSup = obj.FastSLAM.Pt.sup;
            end
            obj.History{time_step}  = template;
        end
    end
end
