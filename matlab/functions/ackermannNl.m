function [delta_fl, delta_fr, delta_rl, delta_rr] = ackermannNl(delta_f,delta_r,d,l)
% Non-linear model of ackermann angle
% Including (inverse)trigonometric functions
% 
% Input
% d: half length of tread
% l: wheel base

x = d / l * (tan(delta_f) - tan(delta_r));
delta_fl = atan2(tan(delta_f) , (1 - x));
delta_fr = atan2(tan(delta_f) , (1 + x));
delta_rl = atan2(tan(delta_r) , (1 - x));
delta_rr = atan2(tan(delta_r) , (1 + x));

end