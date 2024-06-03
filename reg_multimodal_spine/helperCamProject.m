function [surfacePtCloud] = helperCamProject(ptCloud, cameraPosition, cameraTarget, cameraUpDirection, imageSize, focalLength)
%PTPROJECTIONCAM Summary of this function goes here
%   Detailed explanation goes here
% Calculate the center of the point cloud
cloudCenter = mean(ptCloud.Location, 1);
% Set the camera position to be at the top center of the point cloud (+20 on Z)
if nargin <2
    cameraPosition = [cloudCenter(1), cloudCenter(2), cloudCenter(3) + 80];
end

if nargin < 3
    % Set the camera target to be the center of the point cloud
    cameraTarget = cloudCenter;
end

if nargin < 4
    % The camera 'up' direction, typically the Y-axis
    cameraUpDirection = [0, 1, 0];
end

if nargin < 5
    focalLength = 0.5; % Example value, you may need to adjust this
end

if nargin < 6
    imageSize = [1024, 768]; % Example value, typically the resolution of the camera
end

% Create a transformation matrix based on the camera position and target
tform = makeLookAt(cameraPosition, cameraTarget, cameraUpDirection);

% Transform the point cloud to the camera's perspective
ptCloudTransformed = pctransform(ptCloud, affine3d(tform'));

% Set the intrinsic parameters of the camera

principalPoint = imageSize ./ 2; % Assume principal point is at the center of the image
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Create a synthetic depth image from the camera perspective
[depthImage, validIndex] = pcToDepth(ptCloudTransformed, intrinsics);

% Use the validIndex to find the surface points
surfacePoints = ptCloud.Location(validIndex, :);

% Create a point cloud object with the surface points
surfacePtCloud = pointCloud(surfacePoints);
end

function T = makeLookAt(eye, target, up)
    forward = normalize(target - eye);
    right = normalize(cross(forward, up));
    up = cross(right, forward);
    
    T = [right', up', -forward', eye'; 0, 0, 0, 1];
end

% Helper function to normalize a vector
function v = normalize(v)
    v = v ./ norm(v);
end

% Helper function to convert point cloud to depth image
function [depthImage, validIndex] = pcToDepth(ptCloud, intrinsics)
    % Project the points to 2D
    points2D = projectPoints(ptCloud.Location, intrinsics);

    % Initialize depth image
    depthImage = zeros(intrinsics.ImageSize);
    validIndex = false(ptCloud.Count, 1);

    % Loop through each point to create the depth image
    for i = 1:ptCloud.Count
        x = round(points2D(i, 1));
        y = round(points2D(i, 2));
        z = ptCloud.Location(i, 3);
        if x > 0 && y > 0 && x <= intrinsics.ImageSize(2) && y <= intrinsics.ImageSize(1)
            if depthImage(y, x) == 0 || depthImage(y, x) > z
                depthImage(y, x) = z;
                validIndex(i) = true;
            end
        end
    end
end

% Helper function to project 3D points to 2D using camera intrinsics
function points2D = projectPoints(points3D, intrinsics)
    points2D = (points3D(:, 1:2) ./ points3D(:, 3)).* intrinsics.FocalLength;
    points2D = bsxfun(@plus, points2D, intrinsics.PrincipalPoint);
end