function [processedData] = msg_callback_fcn(msg, shape)
%MSG_CALLBACK_FCN Summary of this function goes here
%   Detailed explanation goes here
    persistent matrixFIFO
    if isempty(matrixFIFO)
        matrixFIFO = zeros(shape);
    end
    matrixFIFO(1:end-1, :) = matrixFIFO(2:end, :);
    matrixFIFO(end, :) = msg.pressure;
    
    %% TODO process
    Y = abs(fft(matrixFIFO / shape(1)));
    processedData = Y;
    plot(Y(2:end/2, :))
end