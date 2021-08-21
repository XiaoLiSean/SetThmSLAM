function C = angleIntervalIntersection(A, B)
    % The input set A,B is either a continuous angle interval
    % the output set C is a continuous angle interval
    
    if isempty(A)
        C   = A;
        return
    elseif isempty(B)
        C   = B;
        return
    end
    
    if abs(volume(A) - 2*pi) < deg2rad(0.0001) || volume(A) > 2*pi
        C   = B;
        return
    elseif abs(volume(B) - 2*pi) < deg2rad(0.0001) || volume(B) > 2*pi
        C   = A;
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
        C   = and(C{1}, C{2});
        if isempty(C)
            C1  = intervalWrapToPi(and(A,B));
            C2  = intervalWrapTo2Pi(and(A,B));    
            if length(C1) == 1
                C   = C1;
                return
            elseif length(C2) == 1
                C   = C2;
                return
            else
                warning('Intersection sets number larger than 1')
            end
        end
        warning('Intersection sets number larger than 1')
        A
        B
        C
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