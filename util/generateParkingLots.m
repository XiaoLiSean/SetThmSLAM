function mapLayers = generateParkingLots(carDim, mapDims, numLotsPerRow, occupiedLots, lotSize, ratioMeter2Pixel)

%% This function is used to generate the pixel array for the parking lot   
    
    parkingSpace    = zeros(ceil(mapDims(2)*ratioMeter2Pixel),...
                            ceil(mapDims(1)*ratioMeter2Pixel));
                        
    %% Initialize the boundary as obsticals
    StationaryObstacles                     = parkingSpace + 255;
    StationaryObstacles(2:end-1,2:end-1)    = 0; % Empty interior of the parking space
    
    %% RoadMarkings is not obstical for our test case
    RoadMarkings    = addRoadMarkings(parkingSpace, numLotsPerRow, lotSize, ratioMeter2Pixel);
                                
    %% Initialize the parked cars as obsticals
    ParkedCars  = addCars(parkingSpace, carDim, numLotsPerRow, occupiedLots,...
                          lotSize, ratioMeter2Pixel);
    
    mapLayers.StationaryObstacles   = uint8(StationaryObstacles);
    mapLayers.RoadMarkings          = uint8(RoadMarkings);
    mapLayers.ParkedCars            = uint8(ParkedCars);
end

function RoadMarkings = addRoadMarkings(parkingSpace, numLotsPerRow, lotSize, ratio)
    xLeftCorner     = (size(parkingSpace,2) - numLotsPerRow*lotSize(1)*ratio)/2.0;
    yLeftCorner     = (size(parkingSpace,1) - 2*lotSize(2)*ratio)/2.0;
    colMarkings     = xLeftCorner + ([0:numLotsPerRow])*lotSize(1)*ratio;
    yIdxs           = round(yLeftCorner) + [0:round(2*lotSize(2)*ratio)];
    xIdxs           = round(colMarkings);
    parkingSpace(yIdxs,xIdxs)   = 255;
    parkingSpace(round(mean(yIdxs)), [min(xIdxs):max(xIdxs)])  = 255;
    RoadMarkings                = parkingSpace;
end

function ParkedCars = addCars(parkingSpace, carDim, numLotsPerRow, occupiedLots, lotSize, ratio)
    
    carLengthPixel  = carDim.Length*ratio;
    carWidthPixel   = carDim.Width*ratio;
    xLeftCorner     = (size(parkingSpace,2) - numLotsPerRow*lotSize(1)*ratio)/2.0;
    yLeftCorner     = (size(parkingSpace,1) - 2*lotSize(2)*ratio)/2.0;
    
    for i = 1:size(occupiedLots, 2)
        for j = 1:size(occupiedLots, 1)
            if occupiedLots(j,i) == 1
                xCenter         = xLeftCorner + (i-0.5)*lotSize(1)*ratio;
                yCenter         = yLeftCorner + (j-0.5)*lotSize(2)*ratio;
                parkingSpace    = addCar(parkingSpace, ceil(xCenter), ceil(yCenter),...
                                         ceil(carLengthPixel), ceil(carWidthPixel));
            end
        end
    end
    ParkedCars  = parkingSpace;
end

function ParkedCars = addCar(parkingSpace, xCenter, yCenter, carLengthPixel, carWidthPixel)
    for i = (xCenter-floor(carWidthPixel/2.0)):(xCenter+ceil(carWidthPixel/2.0))
        for j = (yCenter-floor(carLengthPixel/2.0)):(yCenter+ceil(carLengthPixel/2.0))
            parkingSpace(j,i)   = 255;
        end
    end
    ParkedCars  = parkingSpace;
end