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

figure(1)
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
subplot(1, 2, 1)
hold on
himg = imshow(zeros(L,16));
xlabel('channel')
ylabel('time')
subplot(1, 2, 2)
hold on
fimg = imshow(zeros(51,16));
xlabel('channel')
ylabel('frequency')

figure(2)
xlabel('P1')
ylabel('P2')
zlabel('P3')

pause(3)
for i=1:50
    X = tacmat(:, 3:end);
    Y = fft(X);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1, :);
    P1(2:end-1, :) = 2*P1(2:end-1, :);
    [coeff, ~, latent] = pca(P1);
    features = P1 * coeff(:, 1:3);
    
    figure(2)
    hold on
    scatter3(features(:,1), features(:,2), features(:,3), 'x');
    hold off
    
    figure(3)
    hold on
    xlabel('frequncy(Hz)')
    ylabel('|P2(f)|')
    f = 32*(0:(L/2))/L;
    P1(1, :) = 0.0;
    plot(f, P1)
    hold off
    
    norm_dat = (X - baseline)./(5500 - baseline);
    set(himg, 'CData', norm_dat)
    set(fimg, 'CData', P1)
    drawnow
    pause(0.3)
end