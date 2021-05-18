function sectors = decomposeRing2ConvPolygons(center, r_min, r_max, ringSecNum)
% sector the constraint ring to parts as convex polygons
% Input: center --> center of the ring
%        [r_min, r_max] --> lower and upper radius of the ring
%        ringSecNum --> sector the ring to parts of ringSecNum
% Output: sectors --> cell of size ringSecNumx1, each contain a mptPolytope
    sectors     = cell(ringSecNum, 1);
    sector_dt   = 2*pi/ringSecNum;
    for i = 1:ringSecNum
        t_min       = (i-1)*sector_dt;
        t_max       = i*sector_dt;
        t_mid       = t_min + (t_max - t_min)/2.0; % Angle bisector line
        r_mid       = r_max / cos((t_max - t_min)/2.0);
        vertices    = [r_min*cos(t_min), r_min*sin(t_min);
                       r_min*cos(t_max), r_min*sin(t_max);
                       r_max*cos(t_min), r_max*sin(t_min);
                       r_mid*cos(t_mid), r_mid*sin(t_mid);
                       r_max*cos(t_max), r_max*sin(t_max)] + center;
        sectors{i}  = mptPolytope(vertices);
    end 
end
   