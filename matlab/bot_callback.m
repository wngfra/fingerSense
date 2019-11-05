function [] = bot_callback(msg)
%BOT_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global botmat

if isempty(botmat)
    botmat = zeros(100, 24);
end
stamp = msg.header.stamp;
data = [stamp.sec, stamp.nanosec, msg.o_t_ee, msg.v_ee];
botmat(2:end, :) = botmat(1:end-1, :);
botmat(1, :) = data;

end

