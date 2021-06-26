classdef SetThmSLAM < handle
    properties
        n; % number of markers
        m; % number of cameras
        
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
        Measurable_R; % Markers within Measurable_R are measurable
        Au; % Au{i}{.} is matching solution matrices for measurement from camera i
        
        %% Variables used to determine the termination of set update
        dVFractionThreshold;
        P_Vol_pre; % volume of P{i} before set update
        Lxy_Vol_pre; % volume of Lxy{i} before set update
        Lt_Vol_pre; % volume of Lt{i} before set update
        
        %% Variables used to update the sets using rigid body constraints
        enableRigidBodyConstraints; % [boolean] enable set update from rigid body contraints
        e_rb; % rigid body uncertainty in [meter]
        constraintArr; % n*n cell stores interval bounds on the rigid body edge length
        ringSecNum; % sector the constraint ring to parts as convex polygons
        
        %% Variables used to reconstruct vehicle state sets
        isReconstruction; % reconstruction and plot the defined vehicle state instead of the markers
        reconRefIdx; % index of referred marker for vehicle heading reconstruction
        Pxy; % pxy \in Pxy
        Pt; % pt \in Pt
        
    end
    
    methods
        function obj = SetThmSLAM(pr, isStereoVision, enableRigidBodyConstraints, isReconstruction, relativeNominalMarkerStates)
            obj.n           = pr.n;
            obj.m           = pr.m;
            for i = 1:obj.n
                obj.P{i}        = mptPolytope(pr.P{i});
            end
            for i = 1:obj.m
                obj.Lxy{i}      = mptPolytope(pr.Lxy{i});
            end
            obj.Omega_L = mptPolytope(pr.Omega_L);
            obj.Omega_P = mptPolytope(pr.Omega_P);
            obj.Lt      = pr.Lt;
            obj.e_va    = pr.e_va;
            obj.e_w     = pr.maxSpeed*pr.propTime*[1;1];
            obj.isStereoVision  = isStereoVision;
            if obj.isStereoVision
                obj.e_vr    = pr.e_vr;
            end
            obj.Measurable_R                = pr.Measurable_R;
            obj.dVFractionThreshold         = pr.dVFractionThreshold;
            obj.enableRigidBodyConstraints  = enableRigidBodyConstraints;
            obj.isReconstruction            = isReconstruction;
            if obj.enableRigidBodyConstraints
                obj.e_rb            = pr.epsilon_rb;
                obj.constraintArr   = obj.initiate_contraint(relativeNominalMarkerStates);
                obj.ringSecNum      = pr.ring_sector_num;
            end
            if obj.isReconstruction
                obj.reconRefIdx     = pr.ref_marker;
                obj.updateReconstructedSets();
            end
            obj.updatePrevVolume();
        end
        
        %% Get measurement and update Matching: data association
        % remain to be developed, currently assign ground true A_hat as the
        % only unique matching solution for Au
        function getMeasureAndMatching(obj, Ma, Mr, A_hat)
            if obj.isStereoVision
                obj.Ma  = Ma;
                obj.Mr  = Mr;
            else
                obj.Ma  = Ma;
            end
            for i = 1:obj.m
                obj.Au{i}{1}    = A_hat{i};
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
                        obj.update_jth_P_by_Mi(i, markers(idx));
                    end
                end
                % set update from rigid body contraints
                if obj.enableRigidBodyConstraints
                    for i = 1:obj.n
                        for j = 1:obj.n
                            if i == j
                                continue;
                            end
                            obj.update_ith_P_by_constraint(i, obj.constraintArr{i,j}, j);
                        end
                    end
                end
                % update recontructed sets for new defined states
                if obj.isReconstruction
                    obj.updateReconstructedSets();
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
        
        function updateReconstructedSets(obj)
            x_min   = 0;
            x_max   = 0;
            y_min   = 0;
            y_max   = 0;
            for i = 1:obj.n
                p_max   = max(obj.P{i}.P.V, [], 1);
                p_min   = min(obj.P{i}.P.V, [], 1);
                x_min   = x_min + p_min(1)/obj.n;
                x_max   = x_max + p_max(1)/obj.n;
                y_min   = y_min + p_min(2)/obj.n;
                y_max   = y_max + p_max(2)/obj.n;
            end
            obj.Pxy     = mptPolytope(interval([x_min;y_min],[x_max;y_max]));
            [betaInf, betaSup]  = atan2OverConvPolygons(obj.P{obj.reconRefIdx}, obj.Pxy);
            obj.Pt      = interval(betaInf, betaSup);
        end
        %% Function used to update the sets from rigid body contraints
        function constraintArr = initiate_contraint(obj, relativeNominalMarkerStates)
            p_hat_rel       = relativeNominalMarkerStates; 
            constraintArr   = cell(obj.n);
            for i = 1:(obj.n-1)
                pi  = p_hat_rel(:,i);
                for j = (i+1):obj.n
                    pj  = p_hat_rel(:,j);
                    rij = norm(pi-pj, 2);
                    constraintArr{i,j}  = interval(rij-obj.e_rb, rij+obj.e_rb);
                    constraintArr{j,i}  = interval(rij-obj.e_rb, rij+obj.e_rb);
                end
            end
        end
        % Update Pi using constraint between pi and pj
        function update_ith_P_by_constraint(obj, i, constraint, j)
            [r, c, ~]   = ExactMinBoundCircle(obj.P{j}.P.V);
            r_min       = max([constraint.inf - r, 0]);
            r_max       = constraint.sup + r;
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
                if dVFraction < -1e-4
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
    end
end