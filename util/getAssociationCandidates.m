function associationCandidates = getAssociationCandidates(n)
    % associationCandidates is a cell of length n
    % associationCandidates{i} is a matrix whose row vectors contain all 
    % the possible marker (1:n) associations to i measurements
    % i.e. associationCandidates{i}(k,:) is one of the solution to A(n,i)
    associationCandidates   = cell(n, 1);
    for measurementNum = 1:n
        associationCandidates{measurementNum}   = [];
        allCombPerms        = [];
        allCombinations     = nchoosek(1:n, measurementNum);
        for ith_comb = 1:size(allCombinations, 1)
            allCombPerms    = [allCombPerms; perms(allCombinations(ith_comb,:))];
        end
        associationCandidates{measurementNum}   = allCombPerms;
    end
end