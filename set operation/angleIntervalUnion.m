function C = angleIntervalUnion(A, B)
    % The input set A,B is either a continuous angle interval
    % the output set C is a continuous angle interval
    if isempty(A)
        C   = B;
        return
    elseif isempty(B)
        C   = A;
        return
    end
    
    if abs(volume(A) - 2*pi) < deg2rad(0.0001) || volume(A) > 2*pi
        C   = interval(0, 2*pi);
        return
    elseif abs(volume(B) - 2*pi) < deg2rad(0.0001) || volume(B) > 2*pi
        C   = interval(0, 2*pi);
        return        
    end
    
    % First convert intervals A,B to ranges of [-pi, pi]
    A1  = intervalWrapToPi(A);
    B1  = intervalWrapToPi(B);
    infimum1    = min([getInf(A1), getInf(B1)]);
    supremum1   = max([getSup(A1), getSup(B1)]);
    candidate1  = interval(infimum1, supremum1);
    % Second convert intervals A,B to ranges of [0, 2*pi]
    A2  = intervalWrapTo2Pi(A);
    B2  = intervalWrapTo2Pi(B);
    infimum2    = min([getInf(A2), getInf(B2)]);
    supremum2   = max([getSup(A2), getSup(B2)]);
    candidate2  = interval(infimum2, supremum2);
    % Choose the one with smallest volume
    if volume(candidate1) > volume(candidate2)
        C   = candidate2;
    else
        C   = candidate1;
    end
end