function sup = getSup(A)
    if length(A) > 1
        sup     = max([A{1}.sup, A{2}.sup]);
    else
        sup     = A.sup;
    end
end