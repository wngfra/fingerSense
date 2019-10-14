clear all
close all

%% Parameters
memoryCapacity = 100;
numOfChannels = 16;
shape = [memoryCapacity, numOfChannels];

%% Generate custom message types
if ~exist("custom_interfaces", 'dir')
    mkdir custom_interfaces/franka_msgs
    folderPath = fullfile(pwd, "custom_interfaces");
    copyfile("../ros2_ws/src/franka_msgs", folderPath + "/franka_msgs");
    ros2genmsg("custom_interfaces");
end

%% Create subscriber node, make sure the publisher is running
node = ros2node("signal_processor");
pause(1)

pub = ros2publisher(node, "/franka_commands", "franka_msgs/FrankaCommand");
sub_tactile = ros2subscriber(node, "/tactile_signals", @(msg) msg_callback(msg, shape, pub));
% sub_robot = ros2subscriber(node, "/robot_states");