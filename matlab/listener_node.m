clear all
close all

%% Parameters
signalSequenceLength = 100; % Length of signal sequence stored
numOfChannels = 16;

%% Generate custom message types
if ~exist("custom_interfaces", 'dir')
    mkdir custom_interfaces/tactile_sensor_msgs
    folderPath = fullfile(pwd, "custom_interfaces");
    copyfile("../ros2_ws/src/tactile_sensor_msgs", folderPath + "/tactile_sensor_msgs");
    ros2genmsg("custom_interfaces");
end

%% Create subscriber node, make sure the publisher is running
node = ros2node("listener");
pause(1)
sub = ros2subscriber(node, "/tactile_signal");

%% Realtime processing
global matrixFIFO
matrixFIFO = zeros(signalSequenceLength, numOfChannels);
sub.NewMessageFcn = @(msg) (holdFIFO(msg.pressure));

%% A callback function for the ros2 subscriber to maintain a FIFO matrix
% Append the new sample to the end
function [] = holdFIFO(new_entry)
    global matrixFIFO
    matrixFIFO(1:end-1, :) = matrixFIFO(2:end, :);
    matrixFIFO(end, :) = new_entry;
end