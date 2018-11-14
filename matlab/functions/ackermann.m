function [delta_fl, delta_fr, delta_rl, delta_rr] = ackermann(delta_f,delta_r,d,l)
% Linear model of ackermann angle
% Not including (inverse)trigonometric functions
% Assume delta_f,r,fl,fr,rl,rr << 1
% then tan, tan-1 ==> 1.
% Input
% d: half length of tread
% l: wheel base


x = d / l .* ((delta_f) - (delta_r));
delta_fl = delta_f ./ (1 - x);
delta_fr = delta_f ./ (1 + x);
delta_rl = delta_r ./ (1 - x);
delta_rr = delta_r ./ (1 + x);

end