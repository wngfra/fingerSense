close all

%% Generate custom message types
if ~exist("custom_interfaces", 'dir')
    mkdir custom_interfaces/franka_msgs
    folderPath = fullfile(pwd, "custom_interfaces");
    copyfile("../ros2_ws/src/franka_msgs", folderPath + "/franka_msgs");
    ros2genmsg("custom_interfaces");
end

%% Create subscriber node, make sure the publisher is running
node = ros2node("visualizer");
pause(1)

h = heatmap(zeros(16,16));
sub = ros2subscriber(node, "/tactile_signals", @(msg) msg_callback(msg, h));
