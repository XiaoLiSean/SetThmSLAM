 classdef FastSLAM < handle
    properties
        gfun;               %Motion model function
        hfun;               %Measurement Model Function
        Q;                  %Sensor Noise
        M;                  %Motion Model Noise (dynamical and function of input)
        n;                  %Number of Particles
        particles;          %Pose of particle
        particle_weight;    %Particle Weight
        mu;
        Sigma;
    end
    
    methods
        function obj = FastSLAM(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = diag([0.0873^2 70^2]);
            % PF parameters
            obj.mu = init.mu;
            obj.Sigma = 10*eye(2);
            obj.n = init.n;
            obj.particles = init.particles;
            obj.particle_weight = init.particle_weight;
        end
        function prediction(obj, u)
            % Draw Points from noise distribution (Velocity motion model)
            for i = 1:obj.n
                obj.particles(:,i) = obj.gfun(obj.particles(:,i), chol(obj.M(u),'lower')*randn(3,1) + u);
                obj.particles(3,i) = wrapToPi(obj.particles(3,i));
            end    
        end
        
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
                
            %Calculate measurement and difference in measurements
            z_hat   = zeros(2, obj.n);
            z       = [wrapToPi(z(1)); z(2)];
            v       = zeros(2, obj.n); % Innovation z - z_hat
            for i = 1:obj.n
                z_hat(:,i)  = obj.hfun(landmark_x, landmark_y, obj.particles(:,i));
                v(:,i)      = z - z_hat(:,i);
                v(1,i)      = wrapToPi(v(1,i));
            end
            %Use mvnpdf to get weight of corresponding measurement
            w       = zeros(2, obj.n); % weight of measurements z - z_hat
            for i = 1:obj.n
                w(i)    = mvnpdf(v(:,i), zeros(2,1), chol(obj.Q,'lower'));
            end
            %Update Weights
            for i = 1:obj.n
                obj.particle_weight(i)  = obj.particle_weight(i) * w(i);
            end
            w_total                 = sum(obj.particle_weight);
            obj.particle_weight     = obj.particle_weight / w_total;
            
            n_t     = obj.n / 3;
            n_eff   = 1 / sum(obj.particle_weight.^2);
            if n_eff < n_t
                obj.resample();
            end
            
            obj.meanAndVariance();
        end 
         
        function resample(obj)
            newSamples = zeros(size(obj.particles));
            newWeight = zeros(size(obj.particle_weight));
            W = cumsum(obj.particle_weight);
            r = rand/obj.n;
            count = 1;
            for j = 1:obj.n
                u = r+(j-1)/obj.n;
                while u > W(count)
                    count = count+1;
                end
                newSamples(:,j) = obj.particles(:,count);
                newWeight(j) = 1/obj.n;
            end
            obj.particles = newSamples;
            obj.particle_weight = newWeight;
        end
        
        function meanAndVariance(obj)
            obj.mu = mean(obj.particles, 2); 
            % orientation is a bit more tricky.
            sinSum = 0;
            cosSum = 0;
            for s = 1:obj.n
                cosSum = cosSum + cos(obj.particles(3,s));
                sinSum = sinSum + sin(obj.particles(3,s));
            end
            obj.mu(3) = atan2(sinSum, cosSum);     
            % Compute covariance.
            zeroMean = obj.particles - repmat(obj.mu, 1, obj.n);
            for s = 1:obj.n
                zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
            end
            
            obj.Sigma = zeroMean * zeroMean' / obj.n;
        end
    end
end