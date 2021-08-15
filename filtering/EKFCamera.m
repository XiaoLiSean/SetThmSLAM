classdef EKFCamera < matlab.mixin.Copyable
    properties
        state;          % Pose Mean
        Sigma;          % Pose Covariance which is used to update state
        M;              % Sensor Noise Covariance
        
        %% Measurement related
        isStereoVision;
        Measurable_R; % % Markers within Measurable_R are measurable
        FoV; % Camera Field of View
    end
    
    methods
        function obj = EKFCamera(sampledCamStates, isStereoVision, pr, ithCamera)
            obj.isStereoVision  = isStereoVision;
            obj.Measurable_R    = pr.Measurable_R;
            obj.FoV             = pr.FoV;
            % measurement noise covariance
            if obj.isStereoVision
                obj.M   = diag([pr.e_va, pr.e_vr].^2);
            else
                obj.M   = (pr.e_va)^2;
            end
            % initial mean and covariance
            obj.state   = sampledCamStates;
            % initialize the Sigma with 1-sigma circle bound the uncertanty
            % set of pr.Lxy{ithCamera}
            area        = volume(pr.Lxy{ithCamera});
            radius      = sqrt(area/pi);
            sigma_xy    = radius;
            sigma_t     = volume(pr.Lt{ithCamera})/2;
            obj.Sigma   = diag([sigma_xy, sigma_xy, sigma_t].^2);
        end
        
        % zt is the measurement and camState is the state of the corresponding
        % camera in 3D which yields p_z the possibility of getting the
        % measurement of z
        function p_z = measurementUpdate(obj, z, markerState, enableCamUpdate)
            [angle, distance, ~]     = measureModel(markerState, obj.state, obj.Measurable_R, obj.FoV);
            if obj.isStereoVision
                z_hat   = [angle; distance];
                z(1)    = wrapToPi(z(1));
            else
                z_hat   = angle;
                z       = wrapToPi(z);
            end
            H       = measureJacobian(markerState, obj.state, [angle; distance], obj.isStereoVision);
            Q       = H*obj.Sigma*H' + obj.M;
            K       = obj.Sigma*H' / Q;
            v       = z - z_hat; % innovation
            v(1)    = wrapToPi(v(1));
            if enableCamUpdate
                % Freeze camera state update
                obj.state   = obj.state + K*v;
                obj.Sigma   = (eye(size(obj.Sigma))-K*H)*obj.Sigma;
            end
            % (Q+Q')/2 in case of asymmetric sigma due to numerical error
            p_z         = mvnpdf(z, z_hat, (Q+Q')/2); % Possibility for getting measurement z = {range, bearing}
        end
    end
end