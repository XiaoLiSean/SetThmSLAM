function output = intervalWrapTo2Pi(input)
    while (input.inf<0)&&(input.sup<0)
        input   = interval(input.inf+2*pi, input.sup+2*pi);
    end
    while (input.inf>2*pi)&&(input.sup>2*pi)
        input   = interval(input.inf-2*pi, input.sup-2*pi);
    end
    
    if abs(volume(input) - 2*pi) < deg2rad(0.0001) || volume(input) > 2*pi
        output  = interval(0, 2*pi);
    elseif (input.inf < 0 && input.sup > 0) || (input.inf < 2*pi && input.sup > 2*pi)
        output{1}   = interval(0, wrapTo2Pi(input.sup));
        output{2}   = interval(wrapTo2Pi(input.inf), 2*pi);
    else
        if wrapTo2Pi(input.inf) > wrapTo2Pi(input.sup)
            error('Lower limit larger than upper limit.');
        end
        output  = interval(wrapTo2Pi(input.inf), wrapTo2Pi(input.sup));
    end
end