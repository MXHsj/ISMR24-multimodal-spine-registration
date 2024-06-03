% ========================================================================
% file name:    showTissueFiducialPcdPairs.m
% desciption:   visualize multiple pcd data side by side
% author:       Xihan Ma
% date:         2023-11-20
% ========================================================================
function showTissueFiducialPcdPairs(tissue1, fiducial1, tissue2, fiducial2, SHOW_FID, ViewPlane)

if nargin < 5
    SHOW_FID = false;
    ViewPlane = nan;
elseif nargin < 6
    ViewPlane = nan;
end

% ========== params ==========
PNT_SIZE = 3;
FID_SIZE = 200;
XAX_COLOR = [0.15 0.15 0.15];
YAX_COLOR = [0.15 0.15 0.15];
ZAX_COLOR = [0.15 0.15 0.15];
FID1_COLOR = repmat([1,0,1], fiducial1.Count, 1);
FID2_COLOR = repmat([0,1,0], fiducial1.Count, 1);
% ============================

fiducial1.Color = FID1_COLOR;   % temporary color assignment for visualization
fiducial2.Color = FID2_COLOR;

hold on
if ~isnan(ViewPlane)
    pcshowpair(tissue1, tissue2, 'MarkerSize', PNT_SIZE, 'ViewPlane', ViewPlane)
    if SHOW_FID
        pcshowpair(fiducial1, fiducial2, 'MarkerSize', FID_SIZE, 'ViewPlane', ViewPlane, 'ColorSource', 'Color')
    end
else
    pcshowpair(tissue1, tissue2, 'MarkerSize', PNT_SIZE)
    if SHOW_FID
        pcshowpair(fiducial1, fiducial2, 'MarkerSize', FID_SIZE)
    end
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
    
    