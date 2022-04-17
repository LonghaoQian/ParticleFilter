% wrap an angle alpha to theta in [-pi pi]
% author: Longhao Qian
% date: 2021 12 12
% input: 
% alpha (1 x 1) : input angle
% output: 
% theta (1 x 1) : output angle
function theta = WrapAnlge(alpha)
    theta = atan2(sin(alpha), cos(alpha));
end