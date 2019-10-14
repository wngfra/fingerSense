function [] = msg_callback(msg, shape, pub)
%MSG_CALLBACK_FCN Summary of this function goes here
%   Detailed explanation goes here
    persistent matrixFIFO
    persistent command
    
    if isempty(matrixFIFO)
        matrixFIFO = zeros(shape);
    end
    if isempty(command)
        command = [0.1, 0, 0, 0, 0, 0];
    end
    
    matrixFIFO(1:end-1, :) = matrixFIFO(2:end, :);
    matrixFIFO(end, :) = msg.pressure;
    
    %% TODO process
    newMsg = ros2message(pub);
    newMsg.header = msg.header;
    newMsg.command = command;
    pub.send(newMsg);
end