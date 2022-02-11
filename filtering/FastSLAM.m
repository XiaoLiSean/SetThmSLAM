 classdef FastSLAM < matlab.mixin.Copyable
    properties
        n; % number of markers
        m; % number of cameras
        s; % Number of Particles
        particles; % particle with field of marker and camera
        weights; % Particle Weight
        mu; % mean of marker and camera states
        Sigma; % variance of marker and camera states  
        markerKinematics; % markerKinematics{i} stores function to propagate uncertainty sets
        
        %% Measurement and data association
        isStereoVision;
        knownDataAssociation;
        associationCandidates; % associationCandidates{i} is a matrix whose row vectors contain all the possible marker associations to i measurements
        enableCamUpdate; % if enable camera state update
        Ma; % Ma{i} angle measurement set for camera i
        Mr; % Mr{i} range measurement set for camera i
        Au; % if known data association: Au{i} is matching solution matrix for measurement from camera i
            % otherwise: Au{i}{k} is Maximum Likelihood (ML) matching solution matrix for measurement from camera in k'th particle
        
        %% Bounded error for uncertainty sets
        e_w; % control input p_hat{i}{k+1} - p_hat{i}{k} is bounded by e_w
        e_steering; % noise of steering control signal in [rad] if ctrl signal is available
        e_velocity; % noise of velocity control signal in [m/s] if ctrl signal is available
        
        %% Variables used to reconstruct vehicle state sets
        isReconstruction; % reconstruction and plot the defined vehicle state instead of the markers
        Pxy; % pxy \in Pxy
        Pt; % pt \in Pt
    end
    
    methods
        function obj = FastSLAM(pr, markerKinematics, isStereoVision, knownDataAssociation, enableCamUpdate, isReconstruction)
            obj.n       = pr.n;
            obj.m       = pr.m;
            obj.s       = pr.particle_num;
            obj.e_w     = pr.e_w;
            obj.e_steering          = pr.e_steering;
            obj.e_velocity          = pr.e_velocity;
            obj.markerKinematics    = markerKinematics;
            obj.isStereoVision      = isStereoVision;
            obj.enableCamUpdate     = enableCamUpdate;
            obj.isReconstruction    = isReconstruction;
            obj.weights             = ones(obj.s,1)/obj.s;            
            obj.knownDataAssociation    = knownDataAssociation;
            if ~obj.knownDataAssociation
                obj.associationCandidates   = getAssociationCandidates(obj.n);
            end
            % marker in 2D using EKF
            for i = 1:obj.n
                samples     = sampleBox(zonotope(pr.P{i}), obj.s);
                for k = 1:obj.s
                    obj.particles{k}.Marker{i}.state    = samples(:,k);
                end
            end
            % Camera in 3D using partical filtering
            for i = 1:obj.m
                samplesXY   = sampleBox(zonotope(pr.Lxy{i}), obj.s);
                samplesT    = sampleBox(zonotope(pr.Lt{i}), obj.s);
                for k = 1:obj.s
                    obj.particles{k}.EKFCamera{i}       = EKFCamera([samplesXY(:,k); samplesT(k)], isStereoVision, pr, i);
                end
            end
            if obj.isReconstruction
                obj.updateReconstructedSets();
            end
            obj.updateMeanAndVariance()
        end
        
        %% Propagation particles
        function propagateParticles(obj)
            noiseRange      = interval([-obj.e_w(1); -obj.e_w(2)], [obj.e_w(1); obj.e_w(2)]);
            noiseSample     = sampleBox(zonotope(noiseRange), obj.s*obj.n);
            for k = 1:obj.s
                for i = 1:obj.n
                    obj.particles{k}.Marker{i}.state    = obj.particles{k}.Marker{i}.state + noiseSample(:,(k-1)*obj.n+i);
                end
            end
        end
        
        function propagateParticlesWithCtrl(obj, alpha_hat, v_hat, dt)
            noiseRange      = interval([-obj.e_steering; -obj.e_velocity], [obj.e_steering; obj.e_velocity]);
            noiseSample     = sampleBox(zonotope(noiseRange), obj.s);
            for k = 1:obj.s
                alpha       = alpha_hat + noiseSample(1, k);
                v           = v_hat + noiseSample(2, k);
                dState      = obj.particles{k}.Marker{2}.state - obj.particles{k}.Marker{1}.state;
                theta_car   = atan2(dState(2), dState(1));
                for i = 1:obj.n
                    obj.particles{k}.Marker{i}.state    = obj.markerKinematics{i}.propagateMarker(obj.particles{k}.Marker{i}.state, alpha, v, dt, theta_car);
                end
            end
        end

        %% update particle weights using measurements and resample
        % we assume known association
        function updateParticles(obj)
            % Loop through m cameras
            for i = 1:obj.m
                if isempty(obj.Au{i})
                    continue
                end
                for k = 1:obj.s
                    markers     = getUpdateableArr(obj.Au{i}{k});
                    for idx = 1:length(markers)
                        j   = markers(idx);
                        q   = find(obj.Au{i}{k}(:,j)); % q'th measurement associated with j'th marker; 
                        if obj.isStereoVision
                            bearing = obj.Ma{i}(q);
                            range   = obj.Mr{i}(q);
                            z       = [bearing; range];
                        else
                            z       = obj.Ma{i}(q);  
                        end
                    % update particle weights with measurement z
                        p_of_z  = obj.particles{k}.EKFCamera{i}.measurementUpdate(z, obj.particles{k}.Marker{j}.state, obj.enableCamUpdate);
                        obj.weights(k)  = obj.weights(k)*p_of_z; 
                    end
                end
            end
            % normalize Weights
            weight_sum      = sum(obj.weights);
            obj.weights     = obj.weights / weight_sum;

            Neff = 1 / sum(obj.weights.^2);
            if Neff < obj.s / 3
                obj.resample();
            end
            if obj.isReconstruction
                obj.updateReconstructedSets();
            end
            obj.updateMeanAndVariance();
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
            if obj.knownDataAssociation
                for i = 1:obj.m                      
                    if isempty(obj.Ma{i})
                        obj.Au{i}   = {};
                        continue
                    end
                    for k = 1:obj.s
                        obj.Au{i}{k}    = A_hat{i};
                    end
                end
            else
                for i = 1:obj.m                        
                    if isempty(obj.Ma{i})
                        obj.Au{i}   = {};
                        continue
                    end
                    for k = 1:obj.s
                        obj.maximumLikelihoodAssociation(i, k);
                    end
                end
            end
        end
        
        % Au{i}{k} is Maximum Likelihood (ML) matching solution 
        % matrix for measurement from camera in k'th particle
        function maximumLikelihoodAssociation(obj, ith_Camera, kth_Particle)
            zNum        = length(obj.Ma{ith_Camera}); % number of measurements
            P           = zeros(zNum, obj.n); % likelihood matrix
            cam         = obj.particles{kth_Particle}.EKFCamera{ith_Camera};
            
            % calculate P(i,j): means the probalibity of i'th measurement
            % be associated with j'th marker
            for j = 1:obj.n
                markerState    = obj.particles{kth_Particle}.Marker{j}.state;
                for i = 1:zNum
                    [angle, distance, ~]     = measureModel(markerState, cam.state, cam.Measurable_R, cam.FoV);                        
                        
                    if obj.isStereoVision
                        z_hat   = [angle; distance];
                        z       = [wrapToPi(obj.Ma{ith_Camera}(i)); obj.Mr{ith_Camera}(i)];
                    else
                        z_hat   = angle;
                        z       = wrapToPi(obj.Ma{ith_Camera}(i));
                    end
                    H       = measureJacobian(markerState, cam.state, [angle; distance], obj.isStereoVision);
                    Q       = H*cam.Sigma*H' + cam.M;
                    P(i,j)  = mvnpdf(z, z_hat, (Q+Q')/2); % Possibility for getting measurement z = {range, bearing}
                end
            end
            
            % Calculate ML solution
            P   = P ./ sum(P,2); % each row normalized to make sure it sums up to 1
            solutionPool    = obj.associationCandidates{zNum};
            solutionML      = [];
            probabilityML   = 0;
            for ith_sol = 1:size(solutionPool, 1)
                p_ith_sol   = 1;
                for ith_z = 1:zNum
                    p_ith_sol   = p_ith_sol*P(ith_z, solutionPool(ith_sol, ith_z));
                end
                if p_ith_sol >= probabilityML
                    probabilityML   = p_ith_sol;
                    solutionML      = solutionPool(ith_sol, :);
                end
            end
            
            obj.Au{ith_Camera}{kth_Particle}    = zeros(zNum, obj.n);
            for ith_z = 1:zNum
                obj.Au{ith_Camera}{kth_Particle}(ith_z, solutionML(ith_z))  = 1;
            end
        end
        
        %% Resample particles
        function resample(obj)
            newSamples  = cell(1, obj.s);
            newWeight   = zeros(size(obj.weights));
            W           = cumsum(obj.weights);
            r           = rand/obj.s;
            count       = 1;
            for j = 1:obj.s
                u   = r+(j-1)/obj.s;
                while u > W(count)
                    count   = count+1;
                end
                newSamples{j}       = obj.particles{count};
                newWeight(j)        = 1/obj.s;
            end
            obj.particles   = newSamples;
            obj.weights     = newWeight;
        end
        
        %% update Mean, Variance and Reconstructed Sets
        function updateMeanAndVariance(obj)
            % Calculate mean
            obj.mu.marker   = zeros(2, obj.n);
            obj.mu.camera   = zeros(3, obj.m);
            sinSum          = zeros(1, obj.m);
            cosSum          = zeros(1, obj.m);          
            for k = 1:obj.s
                for i = 1:obj.n
                    obj.mu.marker(:,i)      = obj.mu.marker(:,i) + obj.particles{k}.Marker{i}.state / obj.s;
                end
                for i = 1:obj.m
                    obj.mu.camera(:,i)      = obj.mu.camera(:,i) + obj.particles{k}.EKFCamera{i}.state / obj.s;
                    cosSum(1,i)             = cosSum(1,i) + cos(obj.particles{k}.EKFCamera{i}.state(3,1));
                    sinSum(1,i)             = sinSum(1,i) + sin(obj.particles{k}.EKFCamera{i}.state(3,1));
                end
            end
            for i = 1:obj.m
                obj.mu.camera(3,i)      = atan2(sinSum(1,i), cosSum(1,i));
            end
            % Compute covariance.
            zeroMean.marker = cell(1, obj.n); % used to calculate covariance
            zeroMean.camera = cell(1, obj.m); % used to calculate covariance
            for i = 1:obj.n
                zeroMean.marker{1,i}    = zeros(2, obj.s);
            end
            for i = 1:obj.m
                zeroMean.camera{1,i}    = zeros(3, obj.s);
            end
            obj.Sigma.marker    = cell(1, obj.n); 
            obj.Sigma.camera    = cell(1, obj.m);
            for k = 1:obj.s
                for i = 1:obj.n
                    zeroMean.marker{1,i}(:,k)   = obj.particles{k}.Marker{i}.state - obj.mu.marker(:,i);
                end
                for i = 1:obj.m
                    zeroMean.camera{1,i}(:,k)   = obj.particles{k}.EKFCamera{i}.state - obj.mu.camera(:,i);
                end
            end
            for i = 1:obj.n
                obj.Sigma.marker{1,i}       = zeroMean.marker{1,i} * zeroMean.marker{1,i}' / obj.s;
            end
            for i = 1:obj.m
                FullCamSigma                = zeroMean.camera{1,i} * zeroMean.camera{1,i}' / obj.s;
                obj.Sigma.camera{1,i}       = FullCamSigma(1:2,1:2);
            end
        end
        
        function updateReconstructedSets(obj)
            points  = zeros(2, obj.s*obj.n);
            P       = cell(1, obj.n);
            for i = 1:obj.n
                for k = 1:obj.s
                    points(:,(i-1)*obj.s+k) = obj.particles{k}.Marker{i}.state;
                end
                P{i}.P.V    = points(:,((i-1)*obj.s+1):i*obj.s)';
            end
            obj.Pxy     = mptPolytope.enclosePoints(points);
            [betaInf, betaSup]  = atan2OverConvPolygons(P{2}, P{1});
            heading1    = interval(betaInf, betaSup);
            [betaInf, betaSup]  = atan2OverConvPolygons(P{4}, P{3});
            heading2    = interval(betaInf, betaSup);
            obj.Pt      = angleIntervalUnion(heading1, heading2);
        end
    end
end