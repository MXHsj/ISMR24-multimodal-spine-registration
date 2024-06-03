function coloredPtCloud = helperColorPointCloud(ptCloud, colorName)
    % Define RGB values for different colors
    colors = struct('red', [255, 0, 0], 'green', [0, 255, 0], ...
                    'blue', [0, 0, 255], 'yellow', [255, 255, 0], ...
                    'purple', [128, 0, 128], 'cyan', [0, 255, 255]);
    
    % If color not specified, choose a random one
    if nargin < 2 || ~isfield(colors, colorName)
        colorNames = fieldnames(colors);
        randomIndex = randi(length(colorNames));
        colorName = colorNames{randomIndex};
    end

    % Get the selected color
    selectedColor = colors.(colorName);

    % Apply the color to the point cloud
    numPoints = size(ptCloud.Location, 1);
    coloredPoints = repmat(uint8(selectedColor), numPoints, 1);

    % Create a new colored point cloud
    coloredPtCloud = pointCloud(ptCloud.Location, 'Color', coloredPoints);
end