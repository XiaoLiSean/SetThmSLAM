function costmap = generateParkingSpaceMap()
    addpath('./Trajectory Planning')
    pr      = params; % Class defined for storing global parameters
    lib     = demoEssentials; % load 3rdparty functions in Trajectory Planning folder
    
    %% Parking Space Map: Build up binary array representation for parking space
    
    mapLayers   = generateParkingLots(pr.carDims, pr.SpaceDim, pr.numLotsPerRow, pr.occupiedLots, pr.lotSize, pr.ratioMeter2Pixel);
    resolution  = pr.SpaceDim(1)/size(mapLayers.StationaryObstacles, 2); % resolution of the occupancy grids in meter
    costmap     = lib.combineMapLayers(mapLayers, resolution);
   
end