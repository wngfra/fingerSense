function [] = msg_callback(msg, h)
%MSG_CALLBACK_FCN Summary of this function goes here
%   Detailed explanation goes here
    vals = msg.data;
    vals = (double(vals) / mean(vals));
    cov = kron(vals', vals) * 100;
    h.ColorData = cov;
    drawnow()
end