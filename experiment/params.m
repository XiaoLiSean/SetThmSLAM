classdef params
    properties        
        %% CCTV parameters
        FoV = 2*pi; % lidar Field of View (https://www.slamtec.com/en/Lidar/A1)
        Measurable_R = 5; % Markers within Measurable_R are measurable (https://www.slamtec.com/en/Lidar/A1)
        m; % Numbers of total lidar
        n; % number of markers
        l_hat; % lidar Nominal State [x (m);y (m);theta (rad)]  
        e_va; % angle measurement noise bound in rad
        e_vr; % range measurement noise bound in rad
        e_w; % not initialized
        maxSpeed; % in meter/sec
        
        %% Uncertainty sets
        Omega_L; % Entire parking space (including camera sets L)
        Omega_P; % Entire parking space (including marker sets P)
        Lxy; % lidars' position: Lxy{i} in 2D
        Lt; % lidars' heading: Lt{i} in 1D
        P; % Markers' position: P{i} in 2D
        
        %% uncertainty set initialization dimension
        epsilon_Lt  = deg2rad(1e-4); % in rad
        epsilon_Lxy = 1e-4; % in meter
        epsilon_P   = 1; % in meter
        dVFractionThreshold     = 0.01; % used to determine the termination of set update
        ring_sector_num         = 8; % sector the constraint ring to parts as convex polygons
        safetyIndex             = 1.0; % ensure safe propagation given the maxSpeed can be wrong
    end
    methods
        function obj = params()
            num_lidar   = 0;
            e_va        = [];
            e_vr        = [];
            maxSpeed    = [];
            fileID      = fopen('calibrationData/calibrationParams.txt','r');
            while true
                newline     = fgetl(fileID);
                if ~ischar(newline)
                  break; 
                end
                data        = split(newline, ',');
                l_hat_i     = [str2double(data{1}); str2double(data{2}); str2double(data{3})];
                obj.l_hat   = [obj.l_hat, l_hat_i];
                e_va        = [e_va, str2double(data{4})];
                e_vr        = [e_vr, str2double(data{5})];
                maxSpeed    = [maxSpeed, str2double(data{6})];
                num_lidar   = num_lidar + 1;
            end
            obj.n       = 1;
            obj.m       = num_lidar;
            obj.e_va    = obj.safetyIndex*max(e_va);
            obj.e_vr    = obj.safetyIndex*max(e_vr);
            obj.maxSpeed    = obj.safetyIndex*max(maxSpeed);
            
            % Later these data should be initialized differently from the 
            % raw/calibrated data given new lidar layout
            obj.Omega_L     = interval([-0.9; -1.5], [1.1; 0.5]);
            obj.Omega_P     = obj.Omega_L;
            for i = 1:obj.n
                obj.P{i}        = obj.Omega_P;
            end
            for i = 1:obj.m
                obj.Lxy{i}      = interval([obj.l_hat(1,i)-obj.epsilon_Lxy; obj.l_hat(2,i)-obj.epsilon_Lxy],...
                                           [obj.l_hat(1,i)+obj.epsilon_Lxy; obj.l_hat(2,i)+obj.epsilon_Lxy]);
                obj.Lt{i}       = interval(obj.l_hat(3,i)-obj.epsilon_Lt, obj.l_hat(3,i)+obj.epsilon_Lt);
            end
        end
    end
end