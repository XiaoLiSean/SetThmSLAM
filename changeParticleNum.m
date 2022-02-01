clear all; clc;
close all;

%% Initialize Parking Space and Visualization
cameraType              = 'stereo'; % ['mono'/'stereo']
knownDataAssociation    = true; % if the measurement to marker associations are known
saveHistory             = false; % if save the simulation history
saveHistoryConcise      = false & saveHistory; % disable save full history of the simulation
usePrevTrajectory       = true; % use previous stored path
isReconstruction        = true; % reconstruction and plot the defined vehicle state instead of the markers
enableCamUpdate         = [false, true]; % enable update camera set/particle
enableCtrlSignal        = [false, true]; % enable pass control signal to propagate sets/particles
% =======================================================
enableSetSLAM           = false;
enableRBConstraints     = false; % [true/false] to enable rigid body constraint in set update
% =======================================================
enableFastSLAM          = true;
% =======================================================
History                 = load('Path.mat').Historys;
plotTimeStep            = 5;
particleNums            = [10, 100, 500, 1000];
Pxys                    = [];

%% Simulation Main
% Draw Trajectory
for k = 1:length(particleNums)
    pr  = params;
    pr.setParticleNum(particleNums(k));
    PV  = ParkingValet(pr, cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM, knownDataAssociation, ...
        enableRBConstraints, isReconstruction, enableCtrlSignal, saveHistory, saveHistoryConcise);
    currentPlotStep = 0;
    trajectory      = zeros(length(History), 2);
    for i = 1:length(History)
        state               = History{i}.p_car;
        trajectory(i,:)     = [state(1), state(2)];
    end            
    plot(trajectory(:,1), trajectory(:,2), 'LineWidth', 3)
    current_time    = 0;
    time_step       = 1;
    epsilon         = 0.9*PV.pr.simLoopDt; % take care of numerical error in computation
    % Simulation loop for every PV.pr.simLoopDt
    while true
        % =====================================================
        % Plan new path if sub-goal is reached
        if time_step > length(History)
            return
        end
        currentPose = History{time_step}.p_car;
        % =====================================================
        % Control Update of Vehicle states and sets
        propRes     = mod(current_time, PV.pr.propTime);
        if (propRes < epsilon || PV.pr.propTime - propRes < epsilon) && current_time ~= 0
            pose_car    = [currentPose(1), currentPose(2), rad2deg(currentPose(3))];
            PV.vehicleSim.setVehiclePose(pose_car);
            deltaXY     = PV.updateNominalStates(pose_car);
            if PV.enableSetSLAM
                PV.SetSLAM.propagateSets();
            end
            if PV.enableFastSLAM
                if PV.enableCtrlSignalProp(2)
                    PV.FastSLAM.propagateParticlesWithDistance(deltaXY)
                else
                    PV.FastSLAM.propagateParticles();
                end
            end
        end
        % =====================================================
        % Measurement Update Sets
        updateRes   = mod(current_time, PV.pr.updateTime);
        if (updateRes < epsilon || PV.pr.updateTime - updateRes < epsilon) && current_time ~= 0
            if PV.isReconstruction
                PV.updateReconstructedNominalStates()
            end
            PV.updateMeasurements();
            if PV.enableSetSLAM
                PV.SetSLAM.getMeasureAndMatching(PV.Ma, PV.Mr, PV.A_hat);
                PV.SetSLAM.updateSets();
            end
            if PV.enableFastSLAM
                PV.FastSLAM.getMeasureAndMatching(PV.Ma, PV.Mr, PV.A_hat);
                PV.FastSLAM.updateParticles();
            end
        end
        % =====================================================
        % Update Plot and Check if nominal states are in corresponding sets
        plotRes     = mod(current_time, PV.pr.plotTime);
        if plotRes < epsilon || PV.pr.plotTime - plotRes < epsilon
            if currentPlotStep == plotTimeStep
                Pxys    = [Pxys, PV.FastSLAM.Pxy];
                break
            else
                currentPlotStep     = currentPlotStep + 1;
            end
            PV.eraseDrawing();
            PV.drawAll();
            PV.vehicleSim.updatePlot();
        end
        PV.check_guaranteed_property()
        % =====================================================
        % Check if the sub-goal is reached and save history
        if PV.saveHistory
            if PV.saveHistoryConcise
                PV.updateHistoryConcise(time_step);
            else
                PV.updateHistory(time_step);
            end
        end
        current_time    = current_time + PV.pr.simLoopDt;
        time_step       = time_step + 1;
        % =====================================================
    end
    close all;
end


colors      = {'m', 'c', 'g', 'r'};
figure(1)
set(gcf,'color','w');
p_car   = [PV.p_car(1), PV.p_car(2), rad2deg(PV.p_car(3))];
helperPlotVehicle(p_car, pr.carDims, 0, 'Color', 'blue', 'DisplayName', 'trajectory snapshot '); hold on;
for i = 1:length(Pxys)
    plot(Pxys(i), [1,2], 'color', char(colors(i)), 'LineWidth', 4); hold on;
end
axis equal;
set(gca, 'visible', 'off')