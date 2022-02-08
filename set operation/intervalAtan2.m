function angle = intervalAtan2(Y, X)
    
    %======================================================================
    % Case the box of XY is a line or point
    %======================================================================
    if Y.inf == Y.sup && X.inf == X.sup
        angle   = atan2(Y.sup, X.sup);
        angle   = interval(angle, angle);
        return
    elseif Y.inf == Y.sup && X.inf ~= X.sup
        if Y.inf == 0 && X.inf >= 0
            angle   = interval(0, 0);
        elseif Y.inf == 0 && X.sup <= 0
            angle   = interval(pi, pi);
        elseif Y.inf == 0 && X.sup > 0 && X.inf < 0
            error('not support union of intervals [-pi,-pi], [pi,pi]');
        else
            theta1  = atan2(Y.inf, X.inf);
            theta2  = atan2(Y.inf, X.sup);
            theta_inf   = min([theta1, theta2]);
            theta_sup   = max([theta1, theta2]);
            angle       = interval(theta_inf, theta_sup);
        end
        return
    elseif Y.inf ~= Y.sup && X.inf == X.sup
        if X.inf == 0 && Y.inf >= 0
            angle   = interval(pi/2, pi/2);
        elseif X.inf == 0 && Y.sup <= 0
            angle   = interval(-pi/2, -pi/2);
        elseif X.inf == 0 && Y.sup > 0 && Y.inf < 0
            error('not support union of intervals [-pi/2,-pi/2], [pi/2,pi/2]');
        else
            theta1  = atan2(Y.sup, X.inf);
            theta2  = atan2(Y.inf, X.inf);
            theta_inf   = min([theta1, theta2]);
            theta_sup   = max([theta1, theta2]);
            angle       = interval(theta_inf, theta_sup);
        end
        return
    end
    
    %======================================================================
    % Case box shape
    %======================================================================
    %----------------------------------------------------------------------
    % case: 0 is in the interior of box XY;
    %----------------------------------------------------------------------
    if Y.inf < 0 && Y.sup > 0 && X.inf < 0 && X.sup > 0
        angle   = interval(-pi, pi);
    %----------------------------------------------------------------------
    % case: 0 is on the top edge of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf < 0 && Y.sup == 0 && X.inf < 0 && X.sup > 0
        angle   = interval(-pi, 0);
    %----------------------------------------------------------------------
    % case: 0 is on the bottom edge of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf == 0 && Y.sup > 0 && X.inf < 0 && X.sup > 0
        angle   = interval(0, pi);
    %----------------------------------------------------------------------
    % case: 0 is on the right edge of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf < 0 && Y.sup > 0 && X.inf < 0 && X.sup == 0
        angle   = interval(pi/2, 3*pi/2);
    %----------------------------------------------------------------------
    % case: 0 is on the left edge of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf < 0 && Y.sup > 0 && X.inf == 0 && X.sup > 0
        angle   = interval(-pi/2, pi/2);
    %----------------------------------------------------------------------
    % case: 0 is on the left-bottom corner of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf == 0 && Y.sup > 0 && X.inf == 0 && X.sup > 0
        angle   = interval(0, pi/2);        
    %----------------------------------------------------------------------
    % case: 0 is on the left-top corner of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf < 0 && Y.sup == 0 && X.inf == 0 && X.sup > 0
        angle   = interval(-pi/2, 0);
    %----------------------------------------------------------------------
    % case: 0 is on the right-top corner of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf < 0 && Y.sup == 0 && X.inf < 0 && X.sup == 0
        angle   = interval(-pi/2, -pi);
    %----------------------------------------------------------------------
    % case: 0 is on the right-bottom corner of box XY;
    %----------------------------------------------------------------------
    elseif Y.inf == 0 && Y.sup > 0 && X.inf < 0 && X.sup == 0
        angle   = interval(pi/2, pi);
    %----------------------------------------------------------------------
    % case: 0 is outside box XY;
    %---------------------------------------------------------------------- 
    else
        theta1  = atan2(Y.inf, X.inf);
        theta2  = atan2(Y.inf, X.sup);
        theta3  = atan2(Y.sup, X.inf);
        theta4  = atan2(Y.sup, X.sup);
        theta_inf   = min([theta1, theta2, theta3, theta4]);
        theta_sup   = max([theta1, theta2, theta3, theta4]);
        angle       = interval(theta_inf, theta_sup);
    end

end