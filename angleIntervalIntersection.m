function C = angleIntervalIntersection(A, B)
    % The input set A,B is either a subset of [0, 2*pi] or [-pi, pi]
    % the output set C is a subset of [-pi, pi] or [0, 2*pi]
    if A.isempty()
        C   = A;
        return
    elseif B.isempty()
        C   = B;
        return
    end
    
    % First convert intervals A,B to ranges of [-pi, pi]
    A1  = intervalWrapToPi(A);
    B1  = intervalWrapToPi(B);
    candidate1  = getIntersection(A1,B1);
    % Second convert intervals A,B to ranges of [0, 2*pi]
    A2  = intervalWrapTo2Pi(A);
    B2  = intervalWrapTo2Pi(B);
    candidate2  = getIntersection(A2,B2);
    
    if length(candidate2) < length(candidate1)
        C   = candidate2;
    else
        C   = candidate1;
    end
    
    if length(C) == 1
        C   = C{1};
    else
        C
        error('Intersection sets number larger than 1')
    end
end

function C = getIntersection(A_ref,B_ref)
    if length(A_ref) == 1
        A{1}    = A_ref;
    else
        A       = A_ref;
    end
    if length(B_ref) == 1
        B{1}    = B_ref;
    else
        B       = B_ref;
    end
    C   = {};
    for i = 1:length(A)
        for j = 1:length(B)
            candidate   = and(A{i}, B{j});
            if ~isempty(candidate)
                C{length(C)+1}  = candidate;
            end
        end
    end
end