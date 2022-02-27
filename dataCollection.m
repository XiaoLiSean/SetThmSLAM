%% Initialize Parking Space and Visualization
knownDataAssociation    = false; % if the measurement to marker associations are known
saveHistory             = true; % if save the simulation history
saveHistoryConcise      = true & saveHistory; % disable save full history of the simulation
usePrevTrajectory       = true; % use previous stored path
isReconstruction        = true; % reconstruction and plot the defined vehicle state instead of the markers
enableCamUpdate         = [false, true]; % enable update camera set/particle
enableCtrlSignal        = [true, true]; % enable pass control signal to propagate sets/particles
% =======================================================
enableSetSLAM           = true;
enableRBConstraints     = true; % [true/false] to enable rigid body constraint in set update
% =======================================================
enableFastSLAM          = true;
% =======================================================
%% Change Parameters
cameraTypes     = ["mono", "stereo"];
parameters      = params;
num             = 25;
e_vas           = deg2rad(linspace(0.1, 10, num));
e_vrs           = linspace(0.1, 0.5, num);
epsilon_Lts     = deg2rad(linspace(0.1, 10, num));
epsilon_Lxys    = linspace(0.1, 0.5, num);
epsilon_Ps      = linspace(0.1, 5, num);  
e_steerings     = deg2rad(linspace(0.1, 10, num));
e_velocitys     = linspace(0.05, 5, num);  
e_mesh          = {e_vas, e_vrs, epsilon_Lts, epsilon_Lxys, epsilon_Ps, e_steerings, e_velocitys};  

%% Simulation Main
History     = load('Path.mat').Historys;
parameters.plotTime     = inf; % disable the graph update
for k = 1:length(e_mesh)
for cameraType = cameraTypes
    if k == 2 && strcmp(cameraType,'mono') 
        continue
    end
    % =====================================================================
    pr      = {e_vas(1), e_vrs(1), epsilon_Lts(1), epsilon_Lxys(1), epsilon_Ps(1), e_steerings(1), e_velocitys(1)};
    for i = 1:num
        pr{k}   = e_mesh{k}(i);
        [e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity]    = deal(pr{:});
        if exist(getFileName(cameraType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity), 'file') == 2
            disp('Skip '+cameraType+num2str(k)+'_'+num2str(i)+'_'+num2str(e_mesh{k}(i)))
            continue
        else
            disp('Data collection '+cameraType+num2str(k)+'_'+num2str(i)+'_'+num2str(e_mesh{k}(i)))
        end
        parameters.resetParams(e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity);
        PV                      = ParkingValet(parameters, cameraType, enableCamUpdate, enableFastSLAM, enableSetSLAM, knownDataAssociation,...
                                               enableRBConstraints, isReconstruction, enableCtrlSignal, saveHistory, saveHistoryConcise);
        PV.simulateHistory(History);
        saveSimulationHistory(PV, cameraType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity)
    end
end
end
dataCollectionFinished  = true;

%% Support Function
function saveSimulationHistory(PV, cameraType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity)
    filename    = getFileName(cameraType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity);
    Historys    = PV.History;
    save(filename, 'Historys');
end

function filename = getFileName(cameraType, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P, e_steering, e_velocity)
    filename    = "graphData/sensitivity/" + cameraType + "_eva_" + num2str(e_va) + "_evr_" + num2str(e_vr) + ...
                    "_eSteering_" + num2str(e_steering) + "_eVelocity_" + num2str(e_velocity) + ...
                    "_Lt0_" + num2str(epsilon_Lt) + "_Lxy0_" + num2str(epsilon_Lxy) + "_P0_" + num2str(epsilon_P) + '.mat';
end