function [betaInf, betaSup] = atan2OverConvPolygons(P_i, Lxy_i)
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
    
    % atan2 is dis-continuous when the range [betaInf, betaSup] cross point
    % of value PI. In this case, we shall move the [betaInf, betaSup] to
    % range of [0, 2*pi] instead of [-pi, pi]. Note: since the camera is
    % installed outside the bounds. [betaInf, betaSup] has a volume smaller
    % than pi as P_i, Lxy_i cannot intersect with each other
    betaInf     = min(values);
    betaSup     = max(values);
    if betaSup - betaInf >= pi
        values  = wrapTo2Pi(values);
        betaInf = min(values);
        betaSup = max(values);
    end
end