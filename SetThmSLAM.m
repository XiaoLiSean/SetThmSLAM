classdef SetThmSLAM < handle
    properties
        n; % number of markers
        m; % number of cameras
        lxy_hat; % Nominal camera position 
        lt_hat; % Nominal camera heading
        p_hat; % Nominal marker position        
        p_hat_rel; % Nominal marker position in car frame
        p_car; % Nominal current vehicle states
        
        %% Uncertainty sets
        Omega; % Parking space
        Lxy; % Camera position uncertainty set
        Lt; % Camera heading uncertainty set
        P; % marker position uncertainty set
        
        %% Plot handler for three typies of uncertainty sets and Nominal markers
        h_Lxy;
        h_Lt;
        h_P;
        h_p_hat;
    end
    
    methods
        function obj = SetThmSLAM(pr)
            obj.n           = pr.n;
            obj.m           = pr.m;
            obj.p_hat_rel   = pr.p_hat_rel; 
            obj.p_car       = pr.p_0;
            for i = 1:obj.n
                obj.p_hat{i}    = [pr.p_hat(1,i); pr.p_hat(2,i)];
            end
            for i = 1:obj.m
                obj.lxy_hat{i}  = [pr.l_hat(1,i); pr.l_hat(2,i)];
                obj.lt_hat{i}   = pr.l_hat(3,i);
            end
            obj.Omega   = pr.Omega;
            obj.P       = pr.P;
            obj.Lxy     = pr.Lxy;
            obj.Lt      = pr.Lt;
        end
        
        function updateNominalStates(obj, p_car)
            obj.p_car   = [p_car(1); p_car(2); deg2rad(p_car(3))];
            for i = 1:obj.n                
                x_marker_i      = obj.p_car(1) + obj.p_hat_rel(1,i)*cos(obj.p_car(3)) - obj.p_hat_rel(2,i)*sin(obj.p_car(3));
                y_marker_i      = obj.p_car(2) + obj.p_hat_rel(1,i)*sin(obj.p_car(3)) + obj.p_hat_rel(2,i)*cos(obj.p_car(3));
                obj.p_hat{i}    = [x_marker_i; y_marker_i];
            end
        end
        
        function drawSets(obj)
            for i = 1:obj.n
                obj.h_P{i}      = plot(obj.P{i});
                obj.h_p_hat{i}  = plot(obj.p_hat{i}(1), obj.p_hat{i}(2), 'r.', 'MarkerSize', 10);
            end
            for i = 1:obj.m
                obj.h_Lxy{i}    = plot(obj.Lxy{i});   
                %obj.h_Lt{i}     = plot(obj.Lt{i});
            end
        end
        
        function eraseDrawing(obj)
            if isempty(obj.h_P)
                return
            end
            for i = 1:obj.n
                delete(obj.h_P{i})
                delete(obj.h_p_hat{i})
            end
            for i = 1:obj.m
                delete(obj.h_Lxy{i})
                %delete(obj.h_Lt{i})
            end
        end
    end
end