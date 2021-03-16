classdef params
    properties (Constant)
        %% Parking lot parameters
        SpaceDim    = [45, 30]; % Dimension [width/x, height/y] of the parking space in meter
        Omega       = interval([0; 0], params.SpaceDim'); % parking space Set Omega in CORA
        occupiedLots    = [0, 1, 1, 0, 1, 1, 0;
                           1, 0, 1, 1, 0, 0, 1]; % Occupancy array of parking lots
        numLotsPerRow   = size(params.occupiedLots, 2); % Two rows of parking lots in the middle of the map and numLotsPerRow for each row
        lotSize         = [3.5, 6.5]; % Individul parking lot is Dimension [width/x, height/y] in meter
        
        %% Map visualization setting
        ratioMeter2Pixel    = 5; % Increase for higher map resolution
        
        %% Ego RC car parameters (refer to vehicleDimensions in MATLAB Doc.)
        carLength   = 4.7; % car length in meters
        carWidth    = 1.8; % car width in meters
        carHeight   = 1.4; % car height in meters
        RearOverhang    = 1;
        FrontOverhang   = 0.9;
        carDims     = vehicleDimensions(params.carLength, params.carWidth,...
                                        params.carHeight, 'FrontOverhang', params.FrontOverhang,...
                                        'RearOverhang', params.RearOverhang);
    end
end
