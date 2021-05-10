function C = angleIntervalUnion(A, B)
    % The input set A,B is either a subset of [0, 2*pi] or [-pi, pi]
    % the output set C is a subset of [-pi, pi] or [0, 2*pi]
    if A.isempty()
        C   = B;
        return
    elseif B.isempty()
        C   = A;
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