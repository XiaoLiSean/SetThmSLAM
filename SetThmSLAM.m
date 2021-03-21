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
        
        %% Bounded error for uncertainty sets
        e_v; % measurement noise is bounded by e_v
        e_w; % control input p_hat{i}{k+1} - p_hat{i}{k} is bounded by e_w
        
        %% Measurement information
        M; % M{i} measurement set for camera i
        A; % A{i} is association matrix for measurement from camera i
        A_hat; % A_hat{i} is ground true matching solution matrix for measurement from camera i
        Au; % Au{i}{.} is matching solution matrix for measurement from camera i
        FoV; % Camera Field of View
        Measurable_R; % Markers within Measurable_R are measurable
        
        %% Plot handler for three typies of uncertainty sets and Nominal markers and camera heading
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
                obj.P{i}        = mptPolytope(pr.P{i});
            end
            for i = 1:obj.m
                obj.lxy_hat{i}  = [pr.l_hat(1,i); pr.l_hat(2,i)];
                obj.lt_hat{i}   = pr.l_hat(3,i);
                obj.Lxy{i}      = mptPolytope(pr.Lxy{i});
            end
            obj.Omega   = mptPolytope(pr.Omega);
            obj.Lt      = pr.Lt;
            obj.e_v     = pr.e_v;
            obj.e_w     = pr.maxSpeed*pr.sampleTime*[1;1];
            obj.FoV     = pr.FoV;
            obj.Measurable_R    = pr.Measurable_R;
        end
        
        %% Matching: data association
        % remain to be developed, currently assign ground true A_hat as the
        % only unique matching solution for Au
        function matching(obj)
            for i = 1:obj.m
                obj.Au{i}{1}    = obj.A_hat{i};
            end
        end
        
        %% Set propagation by motion
        function propagateSets(obj)
            B_inf   = interval([-1;-1], [1;1]);
            for i = 1:obj.n
                obj.P{i}    = plus(obj.P{i}, mtimes(diag(obj.e_w), B_inf));
            end
        end
        
        %% Set update by measurement (iterative algorithm)
        function updateSets(obj)
            % Loop through m cameras
            for i = 1:obj.m
                if isempty(obj.Au{i})
                    continue
                end
                obj.update_ith_Lt(i);
                obj.update_ith_Lxy(i);
                markers     = obj.getUpdateableArr(i);
                for idx = 1:length(markers)
                    obj.update_jth_P_by_Mi(i, markers(idx))
                end
            end
        end
        
        function update_ith_Lt(obj, i)
            num     = length(obj.Au{i});
            % Loop through solutions for i'th camera measurement
            interval_CUP    = interval(); % union over a all possible matching u
            if isempty(obj.Au{i}{1})
                return
            end
            for u = 1:num
                if isempty(obj.Au{i}{u})
                    continue
                end
                % Loop through measurements for u'th matching solution
                interval_CAP    = interval(-inf, inf); % intersection over a single possible matching u
                for q = 1:length(obj.M{i})
                    j   = find(obj.Au{i}{u}(q,:)); % q'th measurement associated with j'th marker;
                    [betaMin, betaMax]  = atan2OverConvPolygons(obj.P{j}, obj.Lxy{i});
                    lt_est_min      = betaMin - obj.M{i}(q) - obj.e_v;
                    lt_est_max      = betaMax - obj.M{i}(q) - obj.e_v;
                    new_interval    = interval(lt_est_min, lt_est_max);
                    interval_CAP    = and(interval_CAP, new_interval);
                end
                interval_CUP    = or(interval_CUP, interval_CAP);
            end
            obj.Lt{i}   = and(obj.Lt{i}, interval_CUP);
        end

        function update_ith_Lxy(obj, i)
            num     = length(obj.Au{i});
            % Loop through solutions for i'th camera measurement
            interval_CUP    = []; % union over a all possible matching u
            if isempty(obj.Au{i}{1})
                return
            end
            for u = 1:num
                if isempty(obj.Au{i}{u})
                    continue
                end
                % Loop through measurements for u'th matching solution
                interval_CAP    = obj.Omega; % intersection over a single possible matching u
                for q = 1:length(obj.M{i})
                    j   = find(obj.Au{i}{u}(q,:)); % q'th measurement associated with j'th marker;
                    LM  = obj.getConvLM(i, obj.M{i}(q));
                    new_interval    = plus(obj.P{j}, LM);
                    interval_CAP    = and(interval_CAP, new_interval);
                end
                if u == 1
                    interval_CUP    = interval_CAP;
                else
                    interval_CUP    = or(interval_CUP, interval_CAP);
                end
            end
            obj.Lxy{i}  = and(obj.Lxy{i}, interval_CUP);
        end
        
        function LM = getConvLM(obj, i, alpha)
            t_lower     = alpha - obj.e_v + obj.Lt{i}.inf;
            t_upper     = alpha + obj.e_v + obj.Lt{i}.sup;
            mid_t       = t_lower + (t_upper - t_lower)/2.0; % Angle bisector line
            mid_r       = obj.Measurable_R / cos((t_upper - t_lower)/2.0);
            vertices    = [0, -obj.Measurable_R*cos(t_lower), -obj.Measurable_R*cos(t_upper), -mid_r*cos(mid_t);...
                           0, -obj.Measurable_R*sin(t_lower), -obj.Measurable_R*sin(t_upper), -mid_r*sin(mid_t)];
            LM          = mptPolytope(vertices');
        end

        function update_jth_P_by_Mi(obj, i, j)
            num     = length(obj.Au{i});
            % Loop through solutions for i'th camera measurement
            interval_CUP    = [];
            for u = 1:num
                % Loop through measurements for u'th matching solution
                q   = find(obj.Au{i}{u}(:,j)); % q'th measurement associated with j'th marker;
                PM  = obj.getConvPM(i, obj.M{i}(q));
                if u == 1
                    interval_CUP    = PM;
                else
                    interval_CUP    = or(interval_CUP, PM);
                end
            end
            constraint  = plus(obj.Lxy{i}, interval_CUP);
            obj.P{j}    = and(obj.P{j}, constraint);
        end        
        
        function PM = getConvPM(obj, i, alpha)
            t_lower     = alpha - obj.e_v + obj.Lt{i}.inf;
            t_upper     = alpha + obj.e_v + obj.Lt{i}.sup;
            mid_t       = t_lower + (t_upper - t_lower)/2.0; % Angle bisector line
            mid_r       = obj.Measurable_R / cos((t_upper - t_lower)/2.0);
            vertices    = [0, obj.Measurable_R*cos(t_lower), obj.Measurable_R*cos(t_upper), mid_r*cos(mid_t);...
                           0, obj.Measurable_R*sin(t_lower), obj.Measurable_R*sin(t_upper), mid_r*sin(mid_t)];
            PM          = mptPolytope(vertices');
        end
        
        function markers = getUpdateableArr(obj, i)
            markers = [];
            num     = length(obj.Au{i});
            % Loop through solutions for i'th camera measurement
            if isempty(obj.Au{i}{1})
                return
            end
            for u = 1:num
                if isempty(obj.Au{i}{u})
                    continue
                end
                if u == 1
                    [~,markers,~]       = find(obj.Au{i}{u});
                else
                    [~,newMarkers,~]    = find(obj.Au{i}{u});
                    markers             = intersect(markers, newMarkers);
                end
            end
        end
        
        %% Update nominal marker position p_hat using nominal par states p_car
        function updateNominalStates(obj, p_car)
            obj.p_car   = [p_car(1); p_car(2); deg2rad(p_car(3))];
            for i = 1:obj.n
                x_marker_i      = obj.p_car(1) + obj.p_hat_rel(1,i)*cos(obj.p_car(3)) - obj.p_hat_rel(2,i)*sin(obj.p_car(3));
                y_marker_i      = obj.p_car(2) + obj.p_hat_rel(1,i)*sin(obj.p_car(3)) + obj.p_hat_rel(2,i)*cos(obj.p_car(3));
                obj.p_hat{i}    = [x_marker_i; y_marker_i];
            end
        end
        
        %% Obtain measurement from the CCTV system
        function updateMeasurements(obj)
            obj.M       = {};
            obj.A_hat   = {};
            for i = 1:obj.m
                obj.M{i}        = [];
                obj.A_hat{i}    = [];
                for j = 1:obj.n
                    [isMeasurable, alpha]   = obj.isMeasurable(obj.p_hat{j}, obj.lxy_hat{i}, obj.lt_hat{i});
                    if isMeasurable
                        idx             = length(obj.M{i}) + 1;
                        obj.M{i}(idx)   = alpha;
                        obj.A_hat{i}(idx,:)     = zeros(1, obj.n);
                        obj.A_hat{i}(idx,j)     = 1; % j'th marker associated with idx'th alpha measurement
                    end
                end
            end
        end
        
        function [isMeasurable, alpha] = isMeasurable(obj, p, lxy, lt)
            distance    = norm(p-lxy);
            angle_gt    = atan2(p(2)-lxy(2), p(1)-lxy(1)) - lt;
            isMeasurable    = (distance <= obj.Measurable_R) & (abs(angle_gt) <= 0.5*obj.FoV); 
            noise       = -obj.e_v + 2*obj.e_v*rand();
            alpha       = angle_gt + noise;
        end
        
        %% Visualization
        function drawSets(obj)
            for i = 1:obj.n
                obj.h_P{i}      = plot(obj.P{i});
                obj.h_p_hat{i}  = plot(obj.p_hat{i}(1), obj.p_hat{i}(2), 'r.', 'MarkerSize', 10);
            end
            for i = 1:obj.m
                obj.h_Lxy{i}    = plot(obj.Lxy{i});
                t1  = obj.Lt{i}.inf;
                t2  = obj.Lt{i}.sup;
                r   = 5; % line length to visualize the heading uncertainty
                x   = [obj.lxy_hat{i}(1)+r*cos(t1), obj.lxy_hat{i}(1), obj.lxy_hat{i}(1)+r*cos(t2)];
                y   = [obj.lxy_hat{i}(2)+r*sin(t1), obj.lxy_hat{i}(2), obj.lxy_hat{i}(2)+r*sin(t2)];
                obj.h_Lt{i}     = plot(x, y, 'b');
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
                delete(obj.h_Lt{i})
            end
        end
    end
end