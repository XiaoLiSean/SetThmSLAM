function J = measureJacobian(p, camState, z_hat, isStereoVision)
    % z_hat = [bearing, range];
    if isStereoVision
        J   = [(p(2) - camState(2))/(z_hat(2)^2),   (camState(1) - p(1))/(z_hat(2)^2),  -1;
               (camState(1) - p(1))/z_hat(2),       (camState(2) - p(2))/z_hat(2),      0];
    else
        J   = [(p(2) - camState(2))/(z_hat(2)^2),   (camState(1) - p(1))/(z_hat(2)^2),  -1];
    end
end

