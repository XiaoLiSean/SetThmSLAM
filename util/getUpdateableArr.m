%% Function to obtain arr of marker indexes which can be updated with M{i}
function markers = getUpdateableArr(Au_i)
    markers = [];
    num     = length(Au_i);
    % Loop through solutions for i'th camera measurement
    if isempty(Au_i{1})
        return
    end
    for u = 1:num
        if isempty(Au_i{u})
            continue
        end
        if u == 1
            [~,markers,~]       = find(Au_i{u});
        else
            [~,newMarkers,~]    = find(Au_i{u});
            markers             = intersect(markers, newMarkers);
        end
    end
end