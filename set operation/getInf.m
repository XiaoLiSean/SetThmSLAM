function inf = getInf(A)
    if length(A) > 1
        inf     = min([A{1}.inf, A{2}.inf]);
    else
        inf     = A.inf;
    end
end