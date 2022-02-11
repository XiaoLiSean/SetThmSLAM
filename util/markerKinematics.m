classdef markerKinematics < matlab.mixin.Copyable
    properties
        wheelbase; % length of wheelbase in meter
        l_os; % offset length from center of rear wheel axis
        theta_os; % offset angle formed with vehicle orientationorigining at center of rear wheel axis
    end
    methods
        
        function obj = markerKinematics(wheelbase, p_hat_rel)
            obj.wheelbase   = wheelbase; 
            obj.l_os        = sqrt(p_hat_rel(1)^2 + p_hat_rel(2)^2);
            obj.theta_os    = atan2(p_hat_rel(2), p_hat_rel(1));
        end
        
        function p_kp1 = propagateMarker(obj, p_k, alpha, v, dt, theta_car)
            displacement    = v*dt*sqrt((obj.l_os*sin(alpha)/obj.wheelbase)^2+...
                (cos(alpha))^2-obj.l_os*sin(obj.theta_os)*sin(2*alpha)/obj.wheelbase);
            angleShift      = theta_car + obj.theta_os - pi/2 + v*dt*sin(alpha)/(2*obj.wheelbase)+...
                atan2(obj.wheelbase*cos(obj.theta_os), (obj.wheelbase*sin(obj.theta_os)-obj.l_os*tan(alpha)));
            
            p_kp1       = zeros(2,1);
            p_kp1(1)    = p_k(1) + displacement*cos(angleShift);
            p_kp1(2)    = p_k(2) + displacement*sin(angleShift);
        end
        
        function P_kp1 = propagateMarkerSetSLAM(obj, P_k, alpha, v, dt, Pt)
            
            displacement    = v*dt*sqrt((obj.l_os*sin(alpha)/obj.wheelbase)^2+...
                (cos(alpha))^2-obj.l_os*sin(obj.theta_os)*sin(2*alpha)/obj.wheelbase);
            interval_const  = interval(obj.wheelbase*cos(obj.theta_os), obj.wheelbase*cos(obj.theta_os));
            angleShift      = Pt + obj.theta_os - pi/2 + v*dt*sin(alpha)/(2*obj.wheelbase)+...
                intervalAtan2(interval_const, (obj.wheelbase*sin(obj.theta_os)-obj.l_os*tan(alpha)));

            dX          = displacement*cos(angleShift);
            dY          = displacement*sin(angleShift);
            dXY         = zonotope(interval([dX.inf; dY.inf], [dX.sup; dY.sup]));
            P_kp1       = plus(P_k, dXY);
        end
        
    end
end
        