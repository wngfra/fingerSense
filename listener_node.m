clear all
close all

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

global M rid
M = zeros(100, 16);
rid = 0;

sub.NewMessageFcn = @(msg) (holdStack(msg.pressure));

%% Test signals
custom_msg = ros2message("tactile_sensor_msgs/TactileSignal");

function [] = holdStack(new_row)
    global M rid
    if rid == length(M)
        rid = 1;
        plot(M)
    else
        rid = rid + 1;
    end
    M(rid, :) = new_row;
end