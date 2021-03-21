function [betaMin, betaMax] = atan2OverConvPolygons(P_i, Lxy_i)
    % Input: two uncertainty sets
    % Output: interval range of atan2(py-ly, px-lx) 
    % where p = [px;py] in P_i, l=[lx;ly] in Lxy_i
    vertices_p  = P_i.P.V;
    vertices_l  = Lxy_i.P.V;
    values      = [];
    for j = 1:size(vertices_p, 1)
        p   = vertices_p(j,:)';
        for i = 1:size(vertices_l, 1)
            l       = vertices_l(i,:)';
            value   = atan2(p(2)-l(2), p(1)-l(1));
            values  = [values; value];
        end
    end
    betaMin     = min(values);
    betaMax     = max(values);
end