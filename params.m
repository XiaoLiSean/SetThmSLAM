classdef params < matlab.mixin.Copyable
    properties
        %% Parking lot parameters
        SpaceDim    = [45; 30]; % Dimension [width/x, height/y] of the parking space in meter
        occupiedLots    = [0, 1, 1, 0, 1, 1;
                           1, 0, 1, 1, 0, 0]; % Occupancy array of parking lots
        lotSize         = [3.5, 5.5]; % Individul parking lot is Dimension [width/x, height/y] in meter
        maxSpeed        = 5; % Maximum speed at the parking lot (m/sec)
        % note: the maxSpeed is not a hard constraint in the parking valet
        % library, sometimes it will exceed the maxSpeed which will cause
        % unwanted behavior and the nominal states will no longer contained
        % in the corresponding sets. Therefore, a safty index is adopted in
        % later define of e_w
        
        
        %% Map visualization setting
        ratioMeter2Pixel    = 5; % Increase for higher map resolution
        
        %% CCTV parameters
        FoV     = 70*pi/180; % Camera Field of View
        l_hat   = [0, -2, 55*pi/180; 10, -2, 7*pi/18; 22.5, -2, pi/2; 35, -2, 11*pi/18; 45, -2, 125*pi/180;
                   15, 10, -pi/2; 22.5, 10, -pi/2; 30, 10, -pi/2; 47, 10, pi; 47, 20, pi; 32, 15, 0;
                   0, 32, -55*pi/180; 10, 32, -7*pi/18; 22.5, 32, -pi/2; 35, 32, -11*pi/18; 45, 32, -125*pi/180;
                   15, 20, pi/2; 22.5, 20, pi/2; 30, 20, pi/2; -2, 10, 0; -2, 20, 0; 13, 15, -pi;]'; % Camera Nominal State [x (m);y (m);theta (rad)]  
        Measurable_R    = 20; % Markers within Measurable_R are measurable
        e_va            = deg2rad(5); % angle measurement noise bound in rad
        e_vr            = 0.1; % range measurement noise bound in rad
        
        %% Simulation time sychronization and management
        simLoopDt   = 0.01; % [sec] time step of each simulation loop (other time constant should be interger times of this)
        propTime    = 0.01; % [sec] [0.01 default] dt of adjacent set propagation (k --> k+1) during kinametics update (should be the smallest among the four)
        sampleTime  = 0.05; % [sec] [0.05 default] pi longitudinal controller sample time for the integral
        updateTime  = 0.5; % [sec] dt of adjacent measurement set update (k --> k+1)
        plotTime    = 1; % [sec] [n*updateTime, n is interger] time interval to update plot
        
        %% Ego RC car parameters (refer to vehicleDimensions in MATLAB Doc.)
        carLength   = 4.0; % car length in meters
        carWidth    = 1.8; % car width in meters
        carHeight   = 1.4; % car height in meters
        RearOverhang    = 1;
        FrontOverhang   = 0.9;
        p_0             = [10; 5; 0]; % RC car initial state vector [x,y,theta(deg)] defined at the center of rear wheel axis
        maxSteeringAngle    = 35; % in degrees
        safetyIndex         = 1.5; % ensure safe propagation given the maxSpeed can be wrong

        %% Path planning
        numCircles          = 4; % Collision checker refer to (vehicleCostmap and CollisionChecker in MATLAB Doc.)
        approxSeparation    = 0.1; % Specify number of poses to return using a separation of approximately 0.1 m
        
        %% Constant used in initializing and Update Uncertainty Set
        epsilon_Lt  = deg2rad(5); % in rad
        epsilon_Lxy = 0.1; % in meter
        epsilon_P   = 0.5; % in meter
        epsilon_rb  = 0.00; % rigid body uncertainty in [meter]
        dVFractionThreshold     = 0.01; % used to determine the termination of set update
        ring_sector_num         = 8; % sector the constraint ring to parts as convex polygons
        
        %% Variables for setting up particle filter
        particle_num    = 100; % number of particles
        
    end
    
    %% Uncertainty set Properties
    properties
        Omega_L; % Entire parking space (including camera sets L)
        Omega_P; % Entire parking space (including marker sets P)
        p_hat; % initial marker's position
        numLotsPerRow; % Two rows of parking lots in the middle of the map and numLotsPerRow for each row
        n; % number of markers
        m; % Numbers of total cameras
        Lxy; % Cameras' position: Lxy{i} in 2D
        Lt; % Cameras' heading: Lt{i} in 1D
        P; % Markers' position: P{i} in 2D
        e_w; % motion update bound
        carDims; % car dimensions 
        Wheelbase;
        MinTurningRadius; % in meters
        p_hat_rel; % initial markers' position in ego car's frame: p_hat_rel(:,i) = [x;y]
        routePlan; % waypoints of trajectory
    end
    methods
        function obj = params()
            % =============================================================
            % initialize vehicle and parking space parameters
            % =============================================================
            obj.Wheelbase           = obj.carLength - obj.RearOverhang - obj.FrontOverhang;
            obj.MinTurningRadius    = obj.Wheelbase/tan(obj.maxSteeringAngle*pi/180); 
            obj.carDims = vehicleDimensions(obj.carLength, obj.carWidth,...
                            obj.carHeight, 'FrontOverhang', obj.FrontOverhang,...
                            'RearOverhang', obj.RearOverhang); 
            obj.p_hat   = zeros(size(obj.p_hat_rel));
            obj.numLotsPerRow   = size(obj.occupiedLots, 2);
            obj.p_hat_rel       = [-0.25*obj.carLength,  0.5*obj.carWidth; 
                                   0.75*obj.carLength,   0.5*obj.carWidth; 
                                   -0.25*obj.carLength,  -0.5*obj.carWidth; 
                                   0.75*obj.carLength,   -0.5*obj.carWidth]';         
            obj.e_w             = obj.safetyIndex*obj.maxSpeed*obj.propTime*[1;1];            
            obj.n       = size(obj.p_hat_rel, 2);        
            obj.m       = size(obj.l_hat, 2); 
            % =============================================================
            % Set path for path planning
            % =============================================================
            WayPoints   = [obj.p_0'; 32, 5, 0; 40, 15, 90; 30, 25, 180; 10, 25, 180; 5, 15, -90; obj.p_0';];
            StartPose   = WayPoints(1:end-1, :);
            EndPose     = WayPoints(2:end, :);
            attrib      = struct('StopLine', false, 'TurnManeuver', false, 'MaxSpeed', obj.maxSpeed, 'EndSpeed', 0.0);
            Attributes  = repmat([attrib], size(StartPose,1), 1);
            obj.routePlan   = table(StartPose, EndPose, Attributes, 'VariableNames', {'StartPose', 'EndPose', 'Attributes'});
            % =============================================================
            % initialize uncertainty sets
            % =============================================================
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
            % =============================================================
            % Entire parking space is constructed to bound all the initial
            % and future uncertainty set
            % =============================================================
            LeftBottom  = [min([min(obj.l_hat(1,:)) - obj.epsilon_Lxy, 0.0]); min([min(obj.l_hat(2,:)) - obj.epsilon_Lxy, 0.0])];
            RightTop    = [max([max(obj.l_hat(1,:)) + obj.epsilon_Lxy, obj.SpaceDim(1)]); max([max(obj.l_hat(2,:)) + obj.epsilon_Lxy, obj.SpaceDim(2)])];
            obj.Omega_L = interval(LeftBottom, RightTop); 
            obj.Omega_P = interval([0; 0], obj.SpaceDim);
        end
        
        function resetParams(obj, e_va, e_vr, epsilon_Lt, epsilon_Lxy, epsilon_P)
            obj.e_va        = e_va;
            obj.e_vr        = e_vr;
            obj.epsilon_Lt  = epsilon_Lt;
            obj.epsilon_Lxy = epsilon_Lxy;
            obj.epsilon_P   = epsilon_P;            
            % =============================================================
            % Re-initialize uncertainty sets
            % =============================================================
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
            % =============================================================
            % Entire parking space is constructed to bound all the initial
            % and future uncertainty set
            % =============================================================
            LeftBottom  = [min([min(obj.l_hat(1,:)) - obj.epsilon_Lxy, 0.0]); min([min(obj.l_hat(2,:)) - obj.epsilon_Lxy, 0.0])];
            RightTop    = [max([max(obj.l_hat(1,:)) + obj.epsilon_Lxy, obj.SpaceDim(1)]); max([max(obj.l_hat(2,:)) + obj.epsilon_Lxy, obj.SpaceDim(2)])];
            obj.Omega_L = interval(LeftBottom, RightTop); 
            obj.Omega_P = interval([0; 0], obj.SpaceDim);
            end
        end
    end
end