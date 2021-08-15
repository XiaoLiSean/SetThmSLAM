%% 3rdparty Supporting Functions
classdef demoEssentials < matlab.mixin.Copyable
    methods
        % *loadParkingLotMapLayers*
        % Load environment map layers for parking lot
        function mapLayers = loadParkingLotMapLayers(obj)
        %loadParkingLotMapLayers
        %   Load occupancy maps corresponding to 3 layers - obstacles, road
        %   markings, and used spots.

        mapLayers.StationaryObstacles = imread('stationary.bmp');
        mapLayers.RoadMarkings        = imread('road_markings.bmp');
        mapLayers.ParkedCars          = imread('parked_cars.bmp');
        end

        %%%
        % *plotMapLayers*
        % Plot struct containing map layers
        function plotMapLayers(obj, mapLayers)
        %plotMapLayers
        %   Plot the multiple map layers on a figure window.

        figure
        cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
        montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN] )
        title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
        end

        %%%
        % *combineMapLayers*
        % Combine map layers into a single costmap
        function costmap = combineMapLayers(obj, mapLayers, res)
        %combineMapLayers
        %   Combine map layers struct into a single vehicleCostmap.

        combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
            mapLayers.ParkedCars;
        combinedMap = im2single(combinedMap);

        % res = 0.5; % meters
        costmap = vehicleCostmap(combinedMap, 'CellSize', res);
        end

        %%%
        % *configurePlanner*
        % Configure path planner with specified settings
        function configurePlanner(obj, pathPlanner, config)
        %configurePlanner
        % Configure the path planner object, pathPlanner, with settings specified
        % in struct config.

        fieldNames = fields(config);
        for n = 1 : numel(fieldNames)
            if ~strcmpi(fieldNames{n}, 'IsParkManeuver')
                pathPlanner.(fieldNames{n}) = config.(fieldNames{n});
            end
        end
        end

        %%%
        % *plotVelocityProfile*
        % Plot speed profile
        function plotVelocityProfile(obj, cumPathLength, refVelocities, maxSpeed)
        %plotVelocityProfile
        % Plot the generated velocity profile

        % Plot reference velocity along length of the path
        plot(cumPathLength, refVelocities, 'LineWidth', 2);

        % Plot a line to display maximum speed
        hold on
        line([0;cumPathLength(end)], [maxSpeed;maxSpeed], 'Color', 'r')
        hold off

        % Set axes limits
        buffer = 2;
        xlim([0 cumPathLength(end)]);
        ylim([0 maxSpeed + buffer])

        % Add labels
        xlabel('Cumulative Path Length (m)');
        ylabel('Velocity (m/s)');

        % Add legend and title
        legend('Velocity Profile', 'Max Speed')
        title('Generated velocity profile')
        end

        %%%
        % *closeFigures*
        function closeFigures(obj)
        % Close all the figures except the simulator visualization

        % Find all the figure objects
        figHandles = findobj('Type', 'figure');
        for i = 1: length(figHandles)
            if ~strcmp(figHandles(i).Name, 'Automated Valet Parking')
                close(figHandles(i));
            end
        end
        end

        %%%
        % *plotParkingManeuver*
        % Display the generated parking maneuver on a costmap
        function plotParkingManeuver(obj, costmap, refPath, currentPose, parkPose)
        %plotParkingManeuver
        % Plot the generated parking maneuver on a costmap.

        % Plot the costmap, without inflated areas
        plot(costmap, 'Inflation', 'off')

        % Plot reference parking maneuver on the costmap
        hold on
        plot(refPath, 'DisplayName', 'Parking Maneuver')

        title('Parking Maneuver')

        % Zoom into parking maneuver by setting axes limits
        lo = min([currentPose(1:2); parkPose(1:2)]);
        hi = max([currentPose(1:2); parkPose(1:2)]);

        buffer = 6; % meters

        xlim([lo(1)-buffer hi(1)+buffer])
        ylim([lo(2)-buffer hi(2)+buffer])
        end
    end
end