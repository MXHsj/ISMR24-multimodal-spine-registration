% ========================================================================
% file name:    preprocess_us_pcd.m
% desciption:   preprocess robotic us pointcloud
% author:       Xiao Zhang, Xihan Ma
% date:         2023-11-22
% ========================================================================
function [pc_us_clean] = preprocess_us_pcd(pc_us)

% ========= params ==========
ORTHO_PROJ_GRID_SIZE = 0.5;     % 0.5 changed depending on how fine-grained you want the grid to be.
REMOV_NEIGHBOR_THRESH = 3.5;    % 3.5 smaller value preserves more structure
KNN_ERODE_THRESH = 5;           % 5
KNN_ERODE_K = 30;               % 30
PC_DENOISE_NUM_NEIGHBOR = 30;   % 30
PC_DENOISE_THRESH = 5;          % 5
% ===========================
% global fid_us_raw
% Shaodow ROI Remove
pc_us_clean = pc_us;
pc_us_clean = helperROIFilter(pc_us_clean, [-inf -20], [-inf inf], [9.2 inf]); % Top left
pc_us_clean = helperROIFilter(pc_us_clean, [30 inf], [-inf inf],  [3.4 inf]); % Top right
pc_us_clean = helperROIFilter(pc_us_clean, [16 inf], [-inf inf],  [6 inf]); % Top right
pc_us_clean = helperROIFilter(pc_us_clean, [-6 0], [-inf inf],  [25 inf]); % Top
pc_us_clean = helperROIFilter(pc_us_clean, [-12 -8], [-inf inf],  [19 inf]); % Top
pc_us_clean = helperROIFilter(pc_us_clean, [40 inf], [-inf inf],  [-2.5 inf]); % Corner
pc_us_clean = helperROIFilter(pc_us_clean, [-20 16], [-inf -76],  [-inf inf]); % Bottom Line

% Orthographic Projection
pc_us_t = helperOrthProject(pc_us_clean, ORTHO_PROJ_GRID_SIZE, 'top');
pc_us_b = helperOrthProject(pc_us_clean, ORTHO_PROJ_GRID_SIZE, 'bottom');

% Upper Structure Extraction
pc_us_clean = helperRemoveNearestPoints(pc_us_b, pc_us_t, REMOV_NEIGHBOR_THRESH);
pc_us_clean = pcdenoise(pc_us_clean, ...
                        'NumNeighbors', PC_DENOISE_NUM_NEIGHBOR, ...
                        'threshold', PC_DENOISE_THRESH, ...
                        'PreserveStructure', true);
pc_us_clean = helperKnnEroding(pc_us_t, pc_us_clean, KNN_ERODE_THRESH, KNN_ERODE_K);

% Grid wise Downsample
pc_us_clean = pcdownsample(pc_us_clean, 'gridAverage', 1);

