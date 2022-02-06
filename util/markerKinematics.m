classdef markerKinematics < matlab.mixin.Copyable
    properties
        wheelbase; % length of wheelbase in meter
        l_os; % offset length from center of rear wheel axis
        theta_os; % offset angle formed with vehicle orientationorigining at center of rear wheel axis
        dt; % discrete time update interval length
        p_hat; % Nominal marker position
    end
    methods
        function obj = markerKinematics(wheelbase, dt, p_hat, p_hat_rel)
            obj.wheelbase   = wheelbase; 
            obj.dt          = dt; 
            obj.p_hat       = p_hat(:);
            obj.l_os        = sqrt(p_hat_rel(1)^2 + p_hat_rel(2)^2);
            obj.theta_os    = atan2(p_hat_rel(2), p_hat_rel(1));
        end
        function propagateMarker(obj, alpha, v, theta_car)
            displacement    = v*obj.dt*sqrt((obj.l_os*sin(alpha)/obj.wheelbase)^2+...
                (cos(alpha))^2-obj.l_os*sin(obj.theta_os)*sin(2*alpha)/obj.wheelbase);
            angleShift      = theta_car + obj.theta_os - pi/2 + v*obj.dt*sin(alpha)/(2*obj.wheelbase)+...
                atan2(obj.wheelbase*cos(obj.theta_os), (obj.wheelbase*sin(obj.theta_os)-obj.l_os*tan(alpha)));
            obj.p_hat(1)    = obj.p_hat(1) + displacement*cos(angleShift);
            obj.p_hat(2)    = obj.p_hat(2) + displacement*sin(angleShift);
        end
    end
end
        