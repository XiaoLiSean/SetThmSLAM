%system("python dataAcquisition.py")
client  = tcpclient('127.0.0.1', 2333);
while true
    if client.NumBytesAvailable>0
       try
           rawData  = readline(client);
           [range, bearing]     = processRawData(rawData);
           return
       catch ME
           clear client
           rethrow(ME)
       end
    end
end

function [range, bearing] = processRawData(rawData)
    data    = str2double(split(rawData, ' '));
    num     = length(data) / 2;
    range   = data(1:num);
    bearing = data(num+1:end)*pi/180;
end