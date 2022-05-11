for k = 1:obj.s
    for i = 1:4
        plot(obj.particles{k}.Marker{i}.state(1), obj.particles{k}.Marker{i}.state(2), 'b.')
    end
end

for k = 1:obj.s
    for i = 1:4
        plot(obj.particles{k}.Marker{i}.state(1), obj.particles{k}.Marker{i}.state(2), 'r.')
    end
end