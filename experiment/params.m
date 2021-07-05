classdef params
    properties        
        %% CCTV parameters
        FoV = 2*pi; % lidar Field of View (https://www.slamtec.com/en/Lidar/A1)
        Measurable_R = 12; % Markers within Measurable_R are measurable (https://www.slamtec.com/en/Lidar/A1)
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
        epsilon_Lt  = deg2rad(1); % in rad
        epsilon_Lxy = 0.05; % in meter
        epsilon_P   = 1; % in meter
        dVFractionThreshold     = 0.01; % used to determine the termination of set update
        ring_sector_num         = 8; % sector the constraint ring to parts as convex polygons
    end
    methods
        function obj = params()
            fileID  = fopen('calibrationData/calibrationParams.txt','r');
            data    = split(fscanf(fileID,'%s'), ',');
            obj.m       = 1;
            obj.n       = 1;
            obj.l_hat   = [str2double(data{1}); str2double(data{2}); str2double(data{3})];
            obj.e_va    = str2double(data{4});
            obj.e_vr    = str2double(data{5});
            obj.maxSpeed    = str2double(data{6});
            
            % Later these data should be initialized differently from the 
            % raw/calibrated data given new lidar layout
            obj.Omega_L     = interval([-2; -2], [2; 2]);
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