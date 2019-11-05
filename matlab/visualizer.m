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

sub_tac = ros2subscriber(node, "/tactile_signals", @(msg) tac_callback(msg));
% sub_bot = ros2subscriber(node, "/robot_states", @(msg) bot_callback(msg));
pause(4)

global tacmat
baseline = zeros(1, 16);
for i=1:10
    baseline = baseline + mean(tacmat(:, 3:end)) / 10.0;
    pause(0.3)
end

figure(1)
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
subplot(1, 2, 1)
hold on
himg = imshow(zeros(100,16));
hold off
subplot(1, 2, 2)
hold on
fimg = imshow(zeros(51,16));
hold off

for i=1:1000
    dat = tacmat(:, 3:end);
    
    fat = abs(fft(dat)/100);
    P1 = fat(1:100/2+1, :);
    P1(2:end-1, :) = 2*P1(2:end-1, :);
    norm_dat = (dat - baseline)./(6000 - baseline);
    set(himg, 'CData', norm_dat)
    set(fimg, 'CData', P1)
    drawnow
    pause(0.3)
end