% ========================================================================
% file name:    polar2cart.m
% desciption:   convert bmode pixel from polar coord. to Cart coord.
% author:       Xihan Ma
% date:         2023-11-13
% ========================================================================
function [x, z] = polar2cart(theta, d, probe)

if nargin < 2
    probe = 'c3hd';
end

switch probe
    case 'c3hd'
        VIRTUE_CENTER = [320, -96]; % (z, x) [pix]
        HEIGHT = 480;
        WIDTH = 640;
    otherwise
        VIRTUE_CENTER = [0, 0];
        disp('INVALID PROBE TYPE')
end

x = round(d*sind(theta) + VIRTUE_CENTER(1));
z = round(d*cosd(theta) + VIRTUE_CENTER(2));

x(x < 1) = 1;
x(x > WIDTH) = WIDTH;

z(z < 1) = 1;
z(z > HEIGHT) = HEIGHT;

