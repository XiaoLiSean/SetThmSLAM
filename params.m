classdef params
    properties (Constant)
        %% Parking lot parameters
        SpaceDim    = [45, 30]; % Dimension [width/x, height/y] of the parking space in meter
        Omega       = interval([0; 0], params.SpaceDim'); % parking space Set Omega in CORA
        occupiedLots    = [0, 1, 1, 0, 1, 1;
                           1, 0, 1, 1, 0, 0]; % Occupancy array of parking lots
        numLotsPerRow   = size(params.occupiedLots, 2); % Two rows of parking lots in the middle of the map and numLotsPerRow for each row
        lotSize         = [3.5, 5.5]; % Individul parking lot is Dimension [width/x, height/y] in meter
        maxSpeed        = 5.0; % Maximum speed at the parking lot (m/sec)
        
        
        %% Map visualization setting
        ratioMeter2Pixel    = 5; % Increase for higher map resolution
        
        %% CCTV parameters
        FoV     = 70*pi/180; % Camera Field of View
        l_hat   = [0,   0,  55*pi/180; 2.5,   0,  35*pi/180; 22.5,   0,  pi/2; 42.5,  0,  145*pi/180; 45,  0,  125*pi/180;
                   0,  30, -55*pi/180; 2.5,  30, -35*pi/180; 22.5,  30, -pi/2; 42.5, 30, -145*pi/180; 45, 30, -125*pi/180]'; % Camera Nominal State [x (m);y (m);theta (rad)]
        sampleTime  = 0.05; % simulation sample time in second k-->k+1
        m           = size(params.l_hat, 2); % Numbers of total cameras
        Measurable_R    = 20; % Markers within Measurable_R are measurable
        
        %% Ego RC car parameters (refer to vehicleDimensions in MATLAB Doc.)
        carLength   = 4.0; % car length in meters
        carWidth    = 1.8; % car width in meters
        carHeight   = 1.4; % car height in meters
        RearOverhang    = 1;
        FrontOverhang   = 0.9;
        carDims     = vehicleDimensions(params.carLength, params.carWidth,...
                                        params.carHeight, 'FrontOverhang', params.FrontOverhang,...
                                        'RearOverhang', params.RearOverhang);
        p_0         = [10; 5; 0]; % RC car initial state vector [x,y,theta] defined at the center of rear wheel axis
        maxSteeringAngle    = 35; % in degrees
        Wheelbase           = params.carLength - params.RearOverhang - params.FrontOverhang;
        MinTurningRadius    = params.Wheelbase/tan(params.maxSteeringAngle*pi/180); % in meters

        %% Path planning
        numCircles  = 4; % Collision checker refer to (vehicleCostmap and CollisionChecker in MATLAB Doc.)
        WayPoints   = [params.p_0'; 32, 5, 0; 40, 15, 90;
                       30, 25, 180; 10, 25, 180; 5, 15, -90; params.p_0';];
        StartPose   = params.WayPoints(1:end-1, :)
        EndPose     = params.WayPoints(2:end, :)
        attrib      = struct('StopLine', false, 'TurnManeuver', false, 'MaxSpeed', params.maxSpeed, 'EndSpeed', 0.0)
        Attributes  = repmat([params.attrib], size(params.StartPose,1), 1)
        routePlan   = table(params.StartPose, params.EndPose, params.Attributes,...
                            'VariableNames', {'StartPose', 'EndPose', 'Attributes'});
        approxSeparation = 0.1; % Specify number of poses to return using a separation of approximately 0.1 m
        
    end  
end
