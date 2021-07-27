function polytope = decomposeCirc2ConvPolygons(center, r, SecNum)
% overbound circle using polytope 
% Input: center --> center of the ring
%        r --> radius of the circle
%        SecNum --> sector the ring to parts of ringSecNum
% Output: overbound polytope 
    sector_dt   = 2*pi/SecNum;
    vertices    = zeros(SecNum, 2);
    for i = 1:SecNum
        t_min       = (i-1)*sector_dt;
        t_max       = i*sector_dt;
        r_mid       = r / cos((t_max - t_min)/2.0);
        vertices(i,:)   = [r_mid*cos(t_max), r_mid*sin(t_max)]+center;
    end
    polytope    = mptPolytope(vertices);
end