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

sub_tac = ros2subscriber(node, "/tactile_signals", @(msg) tac_callback(msg));
% sub_bot = ros2subscriber(node, "/robot_states", @(msg) bot_callback(msg));
pause(4)

global tacmat
L = length(tacmat);
baseline = zeros(1, 16);
for i=1:10
    baseline = baseline + mean(tacmat(:, 3:end)) / 10.0;
    pause(0.3)
end

uiwait(msgbox('Calibration done! Press OK to continue...'));

for i=1:100
    shftsig = tacmat(:, 3:end) - baseline;
    shftsig(abs(shftsig) < 15) = 0;
    stft(shftsig(:, 9), 32)
    view(-45, 65)
    pause(3)
end