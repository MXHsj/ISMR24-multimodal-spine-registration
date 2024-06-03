function surfacePtCloud = helperOrthProject(ptCloud, gridSize, option)
    if nargin < 2
        gridSize = 0.1;
    end
    if nargin < 3
        option = 'top';
    end
    % Create a grid for the orthographic projection. The resolution can be
    % changed depending on how fine-grained you want the grid to be.
    % gridSize = 0.1; % grid size in the units of the point cloud (e.g., meters)
    xRange = min(ptCloud.Location(:,1)):gridSize:max(ptCloud.Location(:,1));
    yRange = min(ptCloud.Location(:,2)):gridSize:max(ptCloud.Location(:,2));
    
    % Initialize a matrix to hold the maximum Z value for each grid cell
    zMax = zeros(length(yRange), length(xRange)) - Inf;
    zMin = zeros(length(yRange), length(xRange)) + Inf;
    
    % Create a 2D grid
    [X, Y] = meshgrid(xRange, yRange);
    
    % Loop through all points in the point cloud to find the maximum Z value
    for i = 1:ptCloud.Count
        x = ptCloud.Location(i,1);
        y = ptCloud.Location(i,2);
        z = ptCloud.Location(i,3);

        % Find the nearest grid indices
        [~, ix] = min(abs(xRange - x));
        [~, iy] = min(abs(yRange - y));

        % Update the maximum and minimum Z value for the grid cell
        zMax(iy, ix) = max(zMax(iy, ix), z);
        zMin(iy, ix) = min(zMin(iy, ix), z);
    end
    
    % Select the surface points based on the option
    if strcmp(option, 'top')
        surfacePoints = [X(zMax > -Inf), Y(zMax > -Inf), zMax(zMax > -Inf)];
    elseif strcmp(option, 'bottom')
        surfacePoints = [X(zMin < Inf), Y(zMin < Inf), zMin(zMin < Inf)];
    else
        error('Invalid option. Choose either "top" or "bottom".');
    end
 
    
    % Create a new point cloud with only the surface points
    surfacePtCloud = pointCloud(surfacePoints);
end

