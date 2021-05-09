classdef params
    properties (Constant)
        %% Parking lot parameters
        SpaceDim    = [45, 30]; % Dimension [width/x, height/y] of the parking space in meter
        occupiedLots    = [0, 1, 1, 0, 1, 1;
                           1, 0, 1, 1, 0, 0]; % Occupancy array of parking lots
        numLotsPerRow   = size(params.occupiedLots, 2); % Two rows of parking lots in the middle of the map and numLotsPerRow for each row
        lotSize         = [3.5, 5.5]; % Individul parking lot is Dimension [width/x, height/y] in meter
        maxSpeed        = 5; % Maximum speed at the parking lot (m/sec)
        
        
        %% Map visualization setting
        ratioMeter2Pixel    = 5; % Increase for higher map resolution
        
        %% CCTV parameters
        FoV     = 70*pi/180; % Camera Field of View
        l_hat   = [0, -2, 55*pi/180; 10, -2, 7*pi/18; 22.5, -2, pi/2; 35, -2, 11*pi/18; 45, -2, 125*pi/180;
                   15, 10, -pi/2; 30, 10, -pi/2; 47, 10, pi; 47, 20, pi; 32, 15, 0;
                   0, 32, -55*pi/180; 10, 32, -7*pi/18; 22.5, 32, -pi/2; 35, 32, -11*pi/18; 45, 32, -125*pi/180;
                   15, 20, pi/2; 30, 20, pi/2; -2, 10, 0; -2, 20, 0; 13, 15, -pi;]'; % Camera Nominal State [x (m);y (m);theta (rad)]  
        m           = size(params.l_hat, 2); % Numbers of total cameras
        Measurable_R    = 20; % Markers within Measurable_R are measurable
        e_va        = deg2rad(1); % angle measurement noise bound in rad
        e_vr        = 0.5; % range measurement noise bound in rad
        
        %% Simulation time sychronization and management
        simLoopDt   = 0.01; % [sec] time step of each simulation loop (other time constant should be interger times of this)
        propTime    = 0.01; % [sec] [0.01 default] dt of adjacent set propagation (k --> k+1) during kinametics update (should be the smallest among the four)
        sampleTime  = 0.05; % [sec] [0.05 default] pi longitudinal controller sample time for the integral
        updateTime  = 0.1; % [sec] dt of adjacent measurement set update (k --> k+1)
        plotTime    = 0.1; % [sec] time interval to update plot
        
        %% Ego RC car parameters (refer to vehicleDimensions in MATLAB Doc.)
        carLength   = 4.0; % car length in meters
        carWidth    = 1.8; % car width in meters
        carHeight   = 1.4; % car height in meters
        RearOverhang    = 1;
        FrontOverhang   = 0.9;
        carDims     = vehicleDimensions(params.carLength, params.carWidth,...
                                        params.carHeight, 'FrontOverhang', params.FrontOverhang,...
                                        'RearOverhang', params.RearOverhang);
        p_0         = [10; 5; 0]; % RC car initial state vector [x,y,theta(deg)] defined at the center of rear wheel axis
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
        approxSeparation    = 0.1; % Specify number of poses to return using a separation of approximately 0.1 m
        
        %% Constant used in initializing Uncertainty Set
        p_hat_rel   = [0,-0.5*params.carWidth; 0,0.5*params.carWidth; 0.5*params.carLength,0]'; % initial markers' position in ego car's frame: p_hat_rel(:,i) = [x;y]
        epsilon_Lt  = deg2rad(1); % in rad
        epsilon_Lxy = 0.1; % in meter
        epsilon_P   = 5; % in meter
        dVFractionThreshold     = 0.01; % used to determine the termination of set update
        
    end
    %% Uncertainty set Properties
    properties
        Omega; % Entire parking space
        p_hat; % initial marker's position
        n; % number of markers
        Lxy; % Cameras' position: Lxy{i} in 2D
        Lt; % Cameras' heading: Lt{i} in 1D
        P; % Markers' position: P{i} in 2D
    end
    methods
        function obj = params()
            obj.n       = size(obj.p_hat_rel, 2);
            obj.p_hat   = 0.0*obj.p_hat_rel;
            theta       = deg2rad(obj.p_0(3));
            for i = 1:obj.n
                obj.p_hat(1,i)  = obj.p_0(1) + obj.p_hat_rel(1,i)*cos(theta) - obj.p_hat_rel(2,i)*sin(theta);
                obj.p_hat(2,i)  = obj.p_0(2) + obj.p_hat_rel(1,i)*sin(theta) + obj.p_hat_rel(2,i)*cos(theta);
                obj.P{i}        = interval([obj.p_hat(1,i)-obj.epsilon_P; obj.p_hat(2,i)-obj.epsilon_P],...
                                           [obj.p_hat(1,i)+obj.epsilon_P; obj.p_hat(2,i)+obj.epsilon_P]);
            end
            for i = 1:obj.m
                obj.Lxy{i}      = interval([obj.l_hat(1,i)-obj.epsilon_Lxy; obj.l_hat(2,i)-obj.epsilon_Lxy],...
                                           [obj.l_hat(1,i)+obj.epsilon_Lxy; obj.l_hat(2,i)+obj.epsilon_Lxy]);
                obj.Lt{i}       = interval(obj.l_hat(3,i)-obj.epsilon_Lt, obj.l_hat(3,i)+obj.epsilon_Lt);
            end
            % Entire parking space is constructed to bound all the initial
            % and future uncertainty set
            LeftBottom  = [min([min(obj.l_hat(1,:)) - obj.epsilon_Lxy, 0.0]); min([min(obj.l_hat(2,:)) - obj.epsilon_Lxy, 0.0])];
            RightTop    = [max([max(obj.l_hat(1,:)) + obj.epsilon_Lxy, obj.SpaceDim(1)]); max([max(obj.l_hat(2,:)) + obj.epsilon_Lxy, obj.SpaceDim(2)])];
            obj.Omega   = interval(LeftBottom, RightTop); 
        end
    end
end
