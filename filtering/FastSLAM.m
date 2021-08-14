 classdef FastSLAM < handle
    properties
        n; % number of markers
        m; % number of cameras
        s; % Number of Particles
        particles; % particle with field of marker and camera
        weights; % Particle Weight
        mu; % mean of marker and camera states
        Sigma; % variance of marker and camera states  
        
        %% Measurement and data association
        isStereoVision;
        enableCamUpdate; % if enable camera state update
        Ma; % Ma{i} angle measurement set for camera i
        Mr; % Mr{i} range measurement set for camera i
        Au; % Au{i}{.} is matching solution matrices for measurement from camera i
        
        %% Bounded error for uncertainty sets
        e_w; % control input p_hat{i}{k+1} - p_hat{i}{k} is bounded by e_w
        
        %% Variables used to reconstruct vehicle state sets
        isReconstruction; % reconstruction and plot the defined vehicle state instead of the markers
        Pxy; % pxy \in Pxy
        Pt; % pt \in Pt
        P;
    end
    
    methods
        function obj = FastSLAM(pr, isStereoVision, enableCamUpdate, isReconstruction)
            obj.n       = pr.n;
            obj.m       = pr.m;
            obj.s       = pr.particle_num;
            obj.e_w     = pr.e_w;
            obj.isStereoVision      = isStereoVision;
            obj.enableCamUpdate     = enableCamUpdate;
            obj.isReconstruction    =isReconstruction;
            obj.weights             = ones(obj.s,1)/obj.s;
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
        
        function propagateParticlesWithDistance(obj, deltaXY)
            noiseRange      = interval([-obj.e_w(1); -obj.e_w(2)], [obj.e_w(1); obj.e_w(2)]);
            noiseSample     = sampleBox(zonotope(noiseRange), obj.s*obj.n);
            for k = 1:obj.s
                for i = 1:obj.n
                    delta   = deltaXY{i} + noiseSample(:,(k-1)*obj.n+i);
                    obj.particles{k}.Marker{i}.state    = obj.particles{k}.Marker{i}.state + delta;
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
                markers     = getUpdateableArr(obj.Au{i});
                for idx = 1:length(markers)
                    j   = markers(idx);
                    q   = find(obj.Au{i}{1}(:,j)); % q'th measurement associated with j'th marker; 
                    if obj.isStereoVision
                        bearing = obj.Ma{i}(q);
                        range   = obj.Mr{i}(q);
                        z       = [bearing; range];
                    else
                        z       = obj.Ma{i}(q);  
                    end
                    % update particle weights with measurement z
                    for k = 1:obj.s
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
            for i = 1:obj.m
                obj.Au{i}{1}    = A_hat{i};
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
            obj.P   = cell(1, obj.n);
            for i = 1:obj.n
                for k = 1:obj.s
                    points(:,(i-1)*obj.s+k) = obj.particles{k}.Marker{i}.state;
                end
                PiPoints    = points(:,((i-1)*obj.s+1):i*obj.s);
                obj.P{i}    = mptPolytope.enclosePoints(PiPoints);
            end
            obj.Pxy     = mptPolytope.enclosePoints(points);
            [betaInf, betaSup]  = atan2OverConvPolygons(obj.P{2}, obj.P{1});
            heading1    = interval(betaInf, betaSup);
            [betaInf, betaSup]  = atan2OverConvPolygons(obj.P{4}, obj.P{3});
            heading2    = interval(betaInf, betaSup);
            obj.Pt      = angleIntervalIntersection(heading1, heading2);
        end
    end
end