% ========================================================================
% file name:    cart2polar.m
% desciption:   convert bmode pixel from Cart coord. to polar coord.
% author:       Xihan Ma
% date:         2023-11-13
% ========================================================================
function [theta, d] = cart2polar(x, z, probe)

if nargin < 2
    probe = 'c3hd';
end

switch probe
    case 'c3hd'
        VIRTUE_CENTER = [320, -96];
    otherwise
        VIRTUE_CENTER = [0, 0];
        disp('INVALID PROBE TYPE')
end

end