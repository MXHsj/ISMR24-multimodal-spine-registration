% ========================================================================
% file name:    segmentBoneSurface.m
% desciption:   extract bone surface from US by sampling in polar Coord.
% author:       Xihan Ma
% date:         2023-11-13
% ========================================================================
function [mask, rectified] = segmentBoneSurface(image, probe, isVis)

if nargin < 2
    probe = 'c3hd';
    isVis = false;
elseif nargin < 3
    isVis = false;
end

switch probe
    case 'c3hd'
        assert((size(image, 1) == 480) && (size(image, 2) == 640), ...
                'image input must be original size')
        % ========== params ==========
        THETA_MIN = -28;        % [deg] -33
        THETA_MAX = 28;         % [deg] 33
        THETA_INTERV = 0.2;     % [deg]
        THETA = THETA_MIN:THETA_INTERV:THETA_MAX;
        
        D_MIN = 180;        % [pix]
        D_MAX = 350;        % [pix]
        D_INTERV = 0.5;       % [pix]
        D = D_MIN:D_INTERV:D_MAX;
        
        BONE_INT_THRESH = 40;
        % ============================
    otherwise
        mask = nan;
        rectified = nan;
        disp('INVALID PROBE TYPE')
        return
end

% ===== polar coord. roi =====
rectified = zeros(length(D), length(THETA), 'uint8');
for th = 1:length(THETA)
    for d = 1:length(D)
        [x, z] = polar2cart(THETA(th), D(d), probe);
        rectified(d, th) = image(round(z), round(x));
    end
end

% ===== bone surface detection =====
mask = zeros(size(image), 'logical');

% % method1: A-scan peak detection
% [peaks, ind] = max(rectified);
% ind(peaks < BONE_INT_THRESH) = nan;
% for col = 1:length(ind)
%     if ~isnan(ind(col)) 
%         [xb, zb] = polar2cart(THETA(col), D(ind(col)), probe);
%         mask(zb, xb) = true;
%     end
% end

% method2: global intensity thresholding
[row, col] = find(rectified >= BONE_INT_THRESH);
for i = 1:length(row)
    [xb, zb] = polar2cart(THETA(col(i)), D(row(i)), probe);
    mask(zb, xb) = true;
end

%% visualization
if isVis
    % ===== polar coord. roi overlay =====
    imagesc(image); colormap gray;
    hold on;
    for d = 1:length(D)
        [xs, zs] = polar2cart(THETA(1), D(d), probe);
        plot(xs, zs, '.r', 'MarkerSize', 3);
        [xe, ze] = polar2cart(THETA(end), D(d), probe);
        plot(xe, ze, '.r', 'MarkerSize', 3);
    end
    for th = 1:length(THETA)
        [xs, zs] = polar2cart(THETA(th), D(1), probe);
        plot(xs, zs, '.r', 'MarkerSize', 3);
        [xe, ze] = polar2cart(THETA(th), D(end), probe);
        plot(xe, ze, '.r', 'MarkerSize', 3);
    end
    hold off
    
    % ===== bone surface overlay =====
    figure() % bone surface in rectified ROI
    imagesc(rectified); colormap gray
    hold on
%     plot(1:length(ind),ind,'.r','MarkerSize',10);
    plot(col,row,'.r','MarkerSize',10);
    hold off
    
    figure()  % bone surface in original image
    overlay = imoverlay(image, mask, 'red');
    imagesc(overlay); colormap gray
end

end
