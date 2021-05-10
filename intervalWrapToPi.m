function output = intervalWrapToPi(input)
    if abs(volume(input) - 2*pi) < deg2rad(0.001)
        output  = interval(-pi, pi);
    elseif (input.inf < pi && input.sup > pi) || (input.inf < -pi && input.sup > -pi)
        output{1}   = interval(-pi, wrapToPi(input.sup));
        output{2}   = interval(wrapToPi(input.inf), pi);
    else
        output  = interval(wrapToPi(input.inf), wrapToPi(input.sup));
    end
end