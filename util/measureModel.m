function [angle, distance, isMeasurable] = measureModel(p, camState, Measurable_R, FoV)
    distance        = norm(p-camState(1:2));
    angle           = wrapToPi(atan2(p(2)-camState(2), p(1)-camState(1)) - camState(3));
    isMeasurable    = (distance <= Measurable_R) & (abs(angle) <= 0.5*FoV); 
end

