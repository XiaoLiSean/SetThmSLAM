%system("python dataAcquisition.py")
close all; clc;
clear all;
client  = tcpclient('127.0.0.1', 2333);

while true
    if client.NumBytesAvailable>0
       try
           rawData  = read(client); % readline is too inefficient
           [range, bearing]     = processRawData(rawData);
           handle   = polarplot(bearing, range, 'r.'); rlim([0 5000]);
           drawnow limitrate
       catch ME
           clear client
           rethrow(ME)
       end
    end
end

function [range, bearing] = processRawData(rawUint8Data)
    cData   = char(rawUint8Data); % convert unit8 array to chars
    sData   = split(cData,newline); % split string to sentences by '\n'
    idx     = length(sData);
    while isempty(sData{idx})
        if idx == 1 % case the lidar scans are all empty
            range   = [];
            bearing = [];
            return
        end
        idx = idx - 1;
    end
    data    = str2double(split(sData{idx}, ' '));
    num     = length(data) / 2;
    range   = data(1:num);
    bearing = deg2rad(data(num+1:end));
end
