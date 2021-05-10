function output = intervalWrapTo2Pi(input)
    if abs(volume(input) - 2*pi) < deg2rad(0.001)
        output  = interval(0, 2*pi);
    elseif input.inf < 0 && input.sup > 0
        output{1}   = interval(0, wrapTo2Pi(input.sup));
        output{2}   = interval(wrapTo2Pi(input.inf), 2*pi);
    else
        output  = interval(wrapTo2Pi(input.inf), wrapTo2Pi(input.sup));
    end
end