function costmap = generateParkingSpaceMap(fig_label)
    addpath('./Trajectory Planning')
    pr      = params; % Class defined for storing global parameters
    lib     = demoEssentials; % load 3rdparty functions in Trajectory Planning folder
    
    %% Parking Space Map: Build up binary array representation for parking space
    
    mapLayers   = generateParkingLots(pr.carDims, pr.SpaceDim, pr.numLotsPerRow, pr.occupiedLots, pr.lotSize, pr.ratioMeter2Pixel);
    resolution  = pr.SpaceDim(1)/size(mapLayers.StationaryObstacles, 2); % resolution of the occupancy grids in meter
    costmap     = lib.combineMapLayers(mapLayers, resolution);
   
    %% Visualization: Map and Cameras
    figure(fig_label)
    plot(costmap, 'Inflation', 'off'); hold on;
    for i = 1:pr.m
        patch   = get_wedge_patch(pr.l_hat(1,i), pr.l_hat(2,i), pr.l_hat(3,i), pr.FoV, pr.Measurable_R);
        fill(patch(1,:), patch(2,:), 'red', 'FaceAlpha', 0.05, 'EdgeAlpha', 0.0);
        plot(pr.l_hat(1,i), pr.l_hat(2,i), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        offset  = (-1)^(pr.l_hat(2,i) < pr.SpaceDim(2))*2;
        text(pr.l_hat(1,i), pr.l_hat(2,i)+offset, num2str(i), 'Color', 'Red', 'FontSize', 20)
    end
end

function patch = get_wedge_patch(x,y,theta,FoV,r)
    theta_l     = theta - 0.5*FoV;
    theta_u     = theta + 0.5*FoV;
    thetas      = theta_l:pi/180:theta_u;
    patch       = [x;y];
    
    for t = thetas
        point   = [x+r*cos(t); y+r*sin(t)];
        patch   = [patch, point];
    end
end