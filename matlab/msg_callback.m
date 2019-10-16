function [] = msg_callback(msg, pub)
%MSG_CALLBACK_FCN Summary of this function goes here
%   Detailed explanation goes here
    persistent matrixFIFO
    
    if isempty(matrixFIFO)
        matrixFIFO = zeros(50, 16);
    end
    
    matrixFIFO(1:end-1, :) = matrixFIFO(2:end, :);
    matrixFIFO(end, :) = msg.pressure;
    
    plot(matrixFIFO);
    
    %% TODO process
    if mean(mean(matrixFIFO)) >  5000
        newMsg = ros2message(pub);
        newMsg.header = msg.header;
        newMsg.command = randn(1, 6) / 20;
        pub.send(newMsg);
    end
end