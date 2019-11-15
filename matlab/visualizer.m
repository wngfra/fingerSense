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


[LoD,HiD] = wfilters('haar','d');

figure(1)
hold on
for i=1:100
    X = tacmat(:, 3:end);
    [cA,cH,cV,cD] = dwt2(X,LoD,HiD,'mode','symh');
    subplot(2,2,1)
    imagesc(cA)
    subplot(2,2,2)
    imagesc(cH)
    subplot(2,2,3)
    imagesc(cV)
    subplot(2,2,4)
    imagesc(cD)
    pause(0.3)
end