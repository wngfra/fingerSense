function [] = tac_callback(msg)
%TAC_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global tacmat

if isempty(tacmat)
    tacmat = zeros(100, 18);
end

stamp = msg.header.stamp;
data = [stamp.sec, stamp.nanosec, msg.data];
tacmat(2:end, :) = tacmat(1:end-1, :);
tacmat(1, :) = data;

end

