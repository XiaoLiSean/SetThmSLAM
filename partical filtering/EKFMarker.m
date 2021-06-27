classdef EKFMarker < handle
    properties
        state;          % Pose Mean
        Sigma;          % Pose Covariance which is used to update state
        M;              % Sensor Noise Covariance
        
        %% Bounded error for uncertainty sets
        e_w; % control input p_hat{i}{k+1} - p_hat{i}{k} is bounded by e_w
        
        %% Measurement related
        isStereoVision;
        Measurable_R; % % Markers within Measurable_R are measurable
        FoV; % Camera Field of View
    end
    
    methods
        function obj = EKFMarker(sampledMarkerStates, isStereoVision, pr, ithMarker)
            obj.isStereoVision  = isStereoVision;
            obj.Measurable_R    = pr.Measurable_R;
            obj.FoV             = pr.FoV;
            obj.e_w             = pr.e_w;
            % measurement noise covariance
            if obj.isStereoVision
                obj.M   = diag([pr.e_va/3, pr.e_vr/3].^2);
            else
                obj.M   = (pr.e_va/3)^2;
            end
            % initial mean and covariance
            obj.state   = sampledMarkerStates;
            % initialize the Sigma with 3-sigma circle bound the uncertanty
            % set of pr.P{iTHmarker}
            area        = volume(pr.P{ithMarker});
            radius      = sqrt(area/pi);
            sigma       = radius/3;
            obj.Sigma   = diag([sigma, sigma].^2);
        end
        
        function randomWalk(obj)
            noise_x     = rand()*2*obj.e_w(1) - obj.e_w(1);
            noise_y     = rand()*2*obj.e_w(2) - obj.e_w(2);
            obj.state   = obj.state + [noise_x; noise_y];
        end
        
        % zt is the measurement and camState is the state of the corresponding
        % camera in 3D which yields p_z the possibility of getting the
        % measurement of z
        function p_z = measurementUpdate(obj, z, camState)
            [angle, distance, isMeasurable]     = measureModel(obj.state, camState, obj.Measurable_R, obj.FoV);
            if ~isMeasurable
                p_z     = 0; % do not trust this particle by setting a zero weight
                return
            end
            if obj.isStereoVision
                z_hat   = [angle; distance];
                z(1)    = wrapToPi(z(1));
            else
                z_hat   = angle;
                z       = wrapToPi(z);
            end
            H       = measureJacobian(obj.state, camState, z_hat, obj.isStereoVision);
            Q       = H*obj.Sigma*H' + obj.M;
            K       = obj.Sigma*H' / Q;
            v       = z - z_hat; % innovation
            obj.state   = obj.state + K*v;
            obj.Sigma   = (eye(2)-K*H)*obj.Sigma;
            % (Q+Q')/2 in case of asymmetric sigma due to numerical error
            p_z         = mvnpdf(z, z_hat, (Q+Q')/2); % Possibility for getting measurement z = {range, bearing}
        end
    end
end