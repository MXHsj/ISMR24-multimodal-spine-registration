% ========================================================================
% file name:    showTissueFiducialPcd.m
% desciption:   visualize tissue pcd data w/ fiducial
% author:       Xihan Ma
% date:         2023-11-20
% ========================================================================
function showTissueFiducialPcd(tissue, fiducial, ViewPlane)

if nargin < 3
    ViewPlane = nan;
end

% ========== params ==========
PNT_SIZE = 3;
FID_SIZE = 200;
XAX_COLOR = [0.15 0.15 0.15];
YAX_COLOR = [0.15 0.15 0.15];
ZAX_COLOR = [0.15 0.15 0.15];
% ============================

hold on
if ~isnan(ViewPlane)
    pcshow(tissue, 'MarkerSize', PNT_SIZE, 'ViewPlane', ViewPlane)
    pcshow(fiducial, 'MarkerSize', FID_SIZE, 'ViewPlane', ViewPlane)
else
    pcshow(tissue, 'MarkerSize', PNT_SIZE)
    pcshow(fiducial, 'MarkerSize', FID_SIZE)
    view(3)
end
hold off

xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')
axis on equal tight
set(gcf, 'color', 'w'); % make background white
set(gca, 'color', 'w', ...
    'XColor', XAX_COLOR, ...
    'YColor', YAX_COLOR, ...
    'ZColor', ZAX_COLOR);

