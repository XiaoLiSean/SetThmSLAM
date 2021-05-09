currentPose     = obj.vehicleSim.getVehiclePose(); % Input to planner should be a row vector
currentVel      = obj.vehicleSim.getVehicleVelocity();

while ~reachedDestination(obj.behavioralPlanner)

    [nextGoal, speedConfig, isReplanNeeded] = obj.pathPlanner(currentPose, currentVel);
    if isReplanNeeded
        continue
    end
    reachGoal = false;

    % Execute control loop
    while ~reachGoal
        % Calculate control signal using stanley(steeringAngle) + pi(accelCmd, decelCmd) controller
        [accelCmd, decelCmd, steeringAngle, direction] = obj.Controller(currentPose, currentVel);
        % =====================================================
        % Simulate the vehicle using the controller outputs
        % =====================================================
        % Note: this drive code only used to update the control
        % command based on current pose and vel; the vehicle
        % states and plotting is parallelly processed in
        % another two threads initialized in HelperVehicleSimulator
        obj.vehicleSim.drive(accelCmd, decelCmd, steeringAngle); 
        obj.vehicleSim.updateKinematics(obj.pr.propTime)
        obj.vehicleSim.updatePlot()
        % =====================================================
        % Set Membership localization main
        % =====================================================
%                     obj.SetSLAM.eraseDrawing() 
%                     obj.SetSLAM.updateNominalStates(currentPose)
%                     obj.SetSLAM.updateMeasurements()
%                     obj.SetSLAM.matching()
%                     obj.SetSLAM.propagateSets()
%                     obj.SetSLAM.updateSets()
%                     obj.SetSLAM.drawSets()
        % =====================================================
        % Get current pose and velocity of the vehicle
        currentPose  = getVehiclePose(obj.vehicleSim);
        currentVel   = getVehicleVelocity(obj.vehicleSim);
        % Check if the vehicle reaches the goal
        reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
    end
end