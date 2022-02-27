clear all; clc;
close all;
tbxmanager restorepath
global dataCollectionFinished
dataCollectionFinished  = false;

while true
    if dataCollectionFinished
        break
    end
    try
        clear all; clc;
        close all;
        dataCollectionFinished  = false;
        dataCollection
    catch ME
        fprintf('Err msg: %s\n', ME.message);
    end
end