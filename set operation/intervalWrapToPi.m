function output = intervalWrapToPi(input)
    while (input.inf<0)&&(input.sup<0)
        input   = interval(input.inf+2*pi, input.sup+2*pi);
    end
    while (input.inf>2*pi)&&(input.sup>2*pi)
        input   = interval(input.inf-2*pi, input.sup-2*pi);
    end
    
    if abs(volume(input) - 2*pi) < deg2rad(0.0001) || volume(input) > 2*pi
        output  = interval(-pi, pi);
    elseif (input.inf < pi && input.sup > pi) || (input.inf < -pi && input.sup > -pi)
        output{1}   = interval(-pi, wrapToPi(input.sup));
        output{2}   = interval(wrapToPi(input.inf), pi);
    else
        if wrapToPi(input.inf) > wrapToPi(input.sup)
            error('Lower limit larger than upper limit.');
        end
        output  = interval(wrapToPi(input.inf), wrapToPi(input.sup));
    end
end