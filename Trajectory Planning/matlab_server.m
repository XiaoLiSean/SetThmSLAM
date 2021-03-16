t       = tcpip('localhost', 50007);
msg     = array2str(refPoses);

fopen(t);
while 1
    fwrite(t, msg);
    bytes = fread(t, [1, t.BytesAvailable]);
    char(bytes)
    if bytes == "done"
        break
    end
end
fclose(t)

function path_str = array2str(path)
    formatSpec = '%.4f';
    path_str = [string(num2str(path(1,1), formatSpec));
                string(num2str(path(1,2), formatSpec));
                string(num2str(path(1,3), formatSpec))];
    for j = 1:size(path,2)
        for i = 2:size(path,1)
            path_str(j,1) = strcat(path_str(j,1), ',', num2str(path(i,j), formatSpec)); 
        end
    end
    path_str = strcat(path_str(1), ';',...
                      path_str(2), ';',...
                      path_str(3), ';');
end
