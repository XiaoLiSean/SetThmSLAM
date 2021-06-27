function J = measureJacobian(p, camState, z_hat, isStereoVision)
    % z_hat = [bearing, range];
    if isStereoVision
        J   = [(camState(2) - p(2))/(z_hat(2)^2),   (p(1) - camState(1))/(z_hat(2)^2);
               (p(1) - camState(1))/z_hat(2),       (p(2) - camState(2))/z_hat(2)];
    else
        J   = [(camState(2) - p(2))/(z_hat(2)^2),   (p(1) - camState(1))/(z_hat(2)^2)];
    end
end

