% If there is a sim_conn connection open, we close it
try
    sim_conn.stop
    clear server
end

% Starting fresh
clc
clear

% MatlabWebSocket source files needed
addpath('C:/MatlabWebSocket/src/')

% Opening the server
sim_conn = classCARND_SIM(4567);

disp('Ready to connect to simulator.');