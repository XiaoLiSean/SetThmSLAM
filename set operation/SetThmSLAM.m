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
        Omega_L; % Entire parking space (including camera sets L)
        Omega_P; % Entire parking space (including marker sets P)
        Lxy; % Camera position uncertainty set
        Lt; % Camera heading uncertainty set
        P; % marker position uncertainty set
        
        %% Bounded error for uncertainty sets
        e_va; % angle measurement noise is bounded by e_va
        e_vr; % range measurement noise is bounded by e_vr
        e_w; % control input p_hat{i}{k+1} - p_hat{i}{k} is bounded by e_w
        
        %% Measurement information
        isStereoVision; 
        Ma; % Ma{i} angle measurement set for camera i
        Mr; % Mr{i} range measurement set for camera i
        A; % A{i} is association matrix for measurement from camera i
        A_hat; % A_hat{i} is ground true matching solution matrix for measurement from camera i
        Au; % Au{i}{.} is matching solution matrices for measurement from camera i
        FoV; % Camera Field of View
        Measurable_R; % Markers within Measurable_R are measurable
        
        %% Plot handler for three typies of uncertainty sets and Nominal markers and camera heading
        h_Lxy;
        h_Lt;
        h_P;
        h_p_hat;
        
        %% Variables used to determine the termination of set update
        dVFractionThreshold;
        P_Vol_pre; % volume of P{i} before set update
        Lxy_Vol_pre; % volume of Lxy{i} before set update
        Lt_Vol_pre; % volume of Lt{i} before set update
        
        %% Variables used to reconstruct vehicle states and sets
        enableRigidBodyConstraints; % [boolean] enable set update from rigid body contraints
        e_rb; % rigid body uncertainty in [meter]
        constraintArr; % n*n cell stores interval bounds on the rigid body edge length
        ringSecNum; % sector the constraint ring to parts as convex polygons
    end
    
    methods
        function obj = SetThmSLAM(pr, cameraType, enableRigidBodyConstraints)
            if strcmp(cameraType,'stereo')
                obj.isStereoVision  = true;
                obj.e_vr            = pr.e_vr;
            elseif strcmp(cameraType,'mono')
                obj.isStereoVision  = false;
            end
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
            obj.Omega_L = mptPolytope(pr.Omega_L);
            obj.Omega_P = mptPolytope(pr.Omega_P);
            obj.Lt      = pr.Lt;
            obj.e_va    = pr.e_va;
            obj.e_w     = pr.maxSpeed*pr.propTime*[1;1];
            obj.FoV     = pr.FoV;
            obj.Measurable_R                = pr.Measurable_R;
            obj.dVFractionThreshold         = pr.dVFractionThreshold;
            obj.enableRigidBodyConstraints  = enableRigidBodyConstraints;
            if obj.enableRigidBodyConstraints
                obj.e_rb            = pr.epsilon_rb;
                obj.constraintArr   = obj.intiate_contraint();
                obj.ringSecNum      = pr.ring_sector_num;
            end
            obj.updatePrevVolume();
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
            obj.updatePrevVolume();
        end
        
        %% Set update by measurement (iterative algorithm)
        function updateSets(obj)
            while true
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
                % set update from rigid body contraints
                if obj.enableRigidBodyConstraints
                    for i = 1:obj.n
                        for j = 1:obj.n
                            if i == j
                                continue;
                            end
                            obj.update_ith_P_by_constraint(i, obj.constraintArr{i,j}, j)
                        end
                    end
                end
                % terminate updating if all the set shrinking fraction < dVFractionThreshold
                if obj.isConvergy()
                    break;
                else
                    obj.updatePrevVolume();
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
                interval_CAP    = interval(-pi, pi); % intersection over a single possible matching u
                for q = 1:length(obj.Ma{i})
                    j   = find(obj.Au{i}{u}(q,:)); % q'th measurement associated with j'th marker;
                    [betaInf, betaSup]  = atan2OverConvPolygons(obj.P{j}, obj.Lxy{i});
                    lt_est_min      = betaInf - obj.Ma{i}(q) - obj.e_va;
                    lt_est_max      = betaSup - obj.Ma{i}(q) + obj.e_va;
                    new_interval    = interval(lt_est_min, lt_est_max);
                    interval_CAP    = angleIntervalIntersection(interval_CAP, new_interval);
                end
                interval_CUP    = angleIntervalUnion(interval_CUP, interval_CAP);
            end
            obj.Lt{i}   = angleIntervalIntersection(obj.Lt{i}, interval_CUP);
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
                interval_CAP    = obj.Omega_L; % intersection over a single possible matching u
                for q = 1:length(obj.Ma{i})
                    j   = find(obj.Au{i}{u}(q,:)); % q'th measurement associated with j'th marker;
                    LM  = obj.getConvLM(i, q);
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
            obj.Lxy{i}  = and(obj.Lxy{i}, obj.Omega_L); % Parking space constraint
        end

        function update_jth_P_by_Mi(obj, i, j)
            num     = length(obj.Au{i});
            % Loop through solutions for i'th camera measurement
            interval_CUP    = [];
            for u = 1:num
                % Loop through measurements for u'th matching solution
                q   = find(obj.Au{i}{u}(:,j)); % q'th measurement associated with j'th marker;
                PM  = obj.getConvPM(i, q);
                if u == 1
                    interval_CUP    = PM;
                else
                    interval_CUP    = or(interval_CUP, PM);
                end
            end
            constraint  = plus(obj.Lxy{i}, interval_CUP);
            obj.P{j}    = and(obj.P{j}, constraint);
            obj.P{j}    = and(obj.P{j}, obj.Omega_P); % Parking space constraint
        end        
        
        %% Function used to update the sets from rigid body contraints
        function constraintArr = intiate_contraint(obj)
            constraintArr   = cell(obj.n);
            for i = 1:(obj.n-1)
                pi  = obj.p_hat_rel(:,i);
                for j = (i+1):obj.n
                    pj  = obj.p_hat_rel(:,j);
                    rij = norm(pi-pj, 2);
                    constraintArr{i,j}  = interval(rij-obj.e_rb, rij+obj.e_rb);
                    constraintArr{j,i}  = interval(rij-obj.e_rb, rij+obj.e_rb);
                end
            end
        end
        % Update Pi using constraint between pi and pj
        function update_ith_P_by_constraint(obj, i, constraint, j)
            [r, c, ~]   = ExactMinBoundCircle(obj.P{j}.P.V);
            r_min       = constraint.inf;
            r_max       = r + constraint.sup;
            sectors     = decomposeRing2ConvPolygons(c, r_min, r_max, obj.ringSecNum);
            is_first_Pi_sector  = true;
            for id = 1:obj.ringSecNum
                Pi_sector   = and(obj.P{i}, sectors{id});
                if Pi_sector.isempty() == 0
                    if is_first_Pi_sector
                        Pi_union    = Pi_sector;
                        is_first_Pi_sector  = false;
                    else
                        Pi_union    = or(Pi_union, Pi_sector); % automatically using convHull to ensure convex property
                    end                    
                end
            end
            obj.P{i}    = Pi_union;
        end
        
        %% Function used to determine if the set update is convergy or not
        % Method: checking if the volumetric descending is below a certain
        % threshold (fraction of original set volume) for all the sets
        function terminated = isConvergy(obj)
            terminated = true;
            for i = 1:obj.n
                dVFraction  = (obj.P_Vol_pre(i) - volume(obj.P{i})) / obj.P_Vol_pre(i);
                if dVFraction < -1e-10
                    warning('Get increasing volume in set updating');
                end
                if dVFraction > obj.dVFractionThreshold
                    terminated  = false;
                    return
                end
            end
            for i = 1:obj.m
                dVFraction_t    = (obj.Lxy_Vol_pre(i) - volume(obj.Lxy{i})) / obj.Lxy_Vol_pre(i);
                dVFraction_xy   = (obj.Lt_Vol_pre(i) - volume(obj.Lt{i})) / obj.Lt_Vol_pre(i);
                if (dVFraction_t < -1e-10) || (dVFraction_xy < -1e-10)
                    warning('Get increasing volume in set updating');
                end
                if (dVFraction_t > obj.dVFractionThreshold) || (dVFraction_xy > obj.dVFractionThreshold)
                    terminated  = false;
                    return
                end
            end
        end
        
        % Update Volume of each sets
        function updatePrevVolume(obj)
            for i = 1:obj.n
                obj.P_Vol_pre(i)    = volume(obj.P{i});
            end
            for i = 1:obj.m
                obj.Lxy_Vol_pre(i)  = volume(obj.Lxy{i});
                obj.Lt_Vol_pre(i)   = volume(obj.Lt{i});
            end
        end
        
        %% Convex approximation of local sets P_M and L_M in camera and marker frame
        function LM = getConvLM(obj, i, q)
            LM          = mptPolytope(-obj.getVertices(i,q));
        end
        
        function PM = getConvPM(obj, i, q)
            PM          = mptPolytope(obj.getVertices(i,q));
        end
        
        function vertices = getVertices(obj, i, q)
            alpha       = obj.Ma{i}(q);
            t_lower     = alpha - obj.e_va + obj.Lt{i}.inf;
            t_upper     = alpha + obj.e_va + obj.Lt{i}.sup;
            mid_t       = t_lower + (t_upper - t_lower)/2.0; % Angle bisector line
            mid_r       = obj.Measurable_R / cos((t_upper - t_lower)/2.0);
            if obj.isStereoVision
                range       = obj.Mr{i}(q);
                r_lower     = max([range - obj.e_vr, 0.0]);
                r_upper     = min([range + obj.e_vr, obj.Measurable_R]);
                r_bound     = r_upper / cos((t_upper - t_lower)/2.0);
                vertices    = [r_lower*cos(t_lower), r_lower*sin(t_lower);
                               r_lower*cos(t_upper), r_lower*sin(t_upper);
                               r_upper*cos(t_lower), r_upper*sin(t_lower);
                               r_bound*cos(mid_t), r_bound*sin(mid_t);
                               r_upper*cos(t_upper), r_upper*sin(t_upper)];
            else
                vertices    = [0, obj.Measurable_R*cos(t_lower), obj.Measurable_R*cos(t_upper), mid_r*cos(mid_t);...
                               0, obj.Measurable_R*sin(t_lower), obj.Measurable_R*sin(t_upper), mid_r*sin(mid_t)]';
            end
        end
        
        %% function used to check if the nominal state is in the corresponding sets
        function check_guaranteed_property(obj)
            for i = 1:obj.n
                if in(obj.P{i}, obj.p_hat{i}) == 0
                    error('nominal state outside the set');
                end
            end
            for i = 1:obj.m
                if in(obj.Lxy{i}, obj.lxy_hat{i}) == 0 ||...
                    (in(obj.Lt{i}, wrapToPi(obj.lt_hat{i})) == 0 && in(obj.Lt{i}, wrapTo2Pi(obj.lt_hat{i})) == 0)
                    error('nominal state outside the set');
                end
            end
        end
        
        %% Function to obtain arr of marker indexes which can be updated with M{i}
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
            obj.Ma      = {};
            obj.A_hat   = {};
            for i = 1:obj.m
                obj.Ma{i}       = [];
                obj.Mr{i}       = [];
                obj.A_hat{i}    = [];
                for j = 1:obj.n
                    [isMeasurable, alpha, range]    = obj.isMeasurable(obj.p_hat{j}, obj.lxy_hat{i}, obj.lt_hat{i});
                    if isMeasurable
                        idx             = length(obj.Ma{i}) + 1;
                        obj.Ma{i}(idx)  = alpha;
                        if obj.isStereoVision
                            obj.Mr{i}(idx)      = range;
                        end
                        obj.A_hat{i}(idx,:)     = zeros(1, obj.n);
                        obj.A_hat{i}(idx,j)     = 1; % j'th marker associated with idx'th alpha measurement
                    end
                end
            end
        end
        
        function [isMeasurable, alpha, range] = isMeasurable(obj, p, lxy, lt)
            distance    = norm(p-lxy);
            angle       = atan2(p(2)-lxy(2), p(1)-lxy(1)) - lt;
            isMeasurable    = (distance <= obj.Measurable_R) & (abs(angle) <= 0.5*obj.FoV); 
            noise_a     = -obj.e_va + 2*obj.e_va*rand();
            alpha       = angle + noise_a;
            range       = nan;
            if obj.isStereoVision
                noise_r = -obj.e_vr + 2*obj.e_vr*rand();
                range   = distance + noise_r;
            end
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
                r   = 0.25*obj.Measurable_R; % line length to visualize the heading uncertainty
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