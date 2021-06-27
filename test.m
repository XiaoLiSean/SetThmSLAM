figure(2)
for k = 1:obj.s
    % marker in 2D using EKF
    pose = obj.particles{k}.EKFMarker{1}.state;
    plot(pose(1), pose(2), 'r.'); hold on;
    %plot(pr.P{1})

    % Camera in 3D using partical filtering
    for i = 1:obj.m
        pose = obj.particles{k}.camPoses(:,i);
        plot(pose(1), pose(2), 'b.')
    end
end

plot(obj.mu.camera(1,:), obj.mu.camera(2,:), 'bx'); hold on;
plot(obj.mu.marker(1,:), obj.mu.marker(2,:), 'ro'); 


for k = 1:obj.s
    % marker in 2D using EKF
    pose = obj.particles{k}.EKFMarker{1}.state;
    plot(pose(1), pose(2), 'r.'); hold on;
end