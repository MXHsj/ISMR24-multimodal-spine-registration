function filteredPtCloud = helperROIFilter(ptCloud, xLimits, yLimits, zLimits, inverse)
    % Validate inputs
    if nargin < 5
        inverse = false;
    end
    if nargin <4
        error('Invalid number of input arguments.');
    end

    % Extract XYZ coordinates of points
    xyzPoints = ptCloud.Location;

    % Filter points within the defined ROI
    isInROI = xyzPoints(:,1) >= xLimits(1) & xyzPoints(:,1) <= xLimits(2) & ...
              xyzPoints(:,2) >= yLimits(1) & xyzPoints(:,2) <= yLimits(2) & ...
              xyzPoints(:,3) >= zLimits(1) & xyzPoints(:,3) <= zLimits(2);

    % Create a new point cloud with points without the ROI
    if inverse
        filteredPtCloud = pointCloud(xyzPoints(isInROI, :));
    else
        filteredPtCloud = pointCloud(xyzPoints(~isInROI, :));
    end
    
end