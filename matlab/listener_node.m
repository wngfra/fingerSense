clear all
close all

%% Parameters
memoryCapacity = 100;
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
global matrixFIFO fftMatrix
matrixFIFO = zeros(memoryCapacity, numOfChannels);
fftMatrix = zeros(memoryCapacity, numOfChannels);
sub.NewMessageFcn = @(msg) (holdFIFO(msg.pressure));

figure(1)
hold on
for i = 1:10000
    fftMatrix = abs(fftMatrix / 16);
    plot(fftMatrix(2:end/2, :))
    pause(0.5)
end

%% A callback function for the ros2 subscriber to maintain a FIFO matrix
% Append the new sample to the end
function [] = holdFIFO(new_entry)
    global matrixFIFO fftMatrix
    matrixFIFO(1:end-1, :) = matrixFIFO(2:end, :);
    matrixFIFO(end, :) = new_entry;
    fftMatrix = fft(matrixFIFO);
end