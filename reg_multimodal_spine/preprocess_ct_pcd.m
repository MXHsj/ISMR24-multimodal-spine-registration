% ========================================================================
% file name:    preprocess_ct_pcd.m
% desciption:   preprocess ct pointcloud
% author:       Xiao Zhang, Xihan Ma
% date:         2023-11-22
% ========================================================================
function [pc_ct_clean] = preprocess_ct_pcd(pc_ct)
% Orthographic Projection
gridsize = 0.1; % changed depending on how fine-grained you want the grid to be.
pc_ct_t = helperOrthProject(pc_ct, gridsize, 'top');
pc_ct_b = helperOrthProject(pc_ct, gridsize, 'bottom');

% Upper Structure Extraction
pc_ct_clean = helperRemoveNearestPoints(pc_ct_b, pc_ct_t, 3);
pc_ct_clean = pcdenoise(pc_ct_clean,"NumNeighbors", 30, "threshold", 5, "PreserveStructure",true);
pc_ct_clean = helperKnnEroding(pc_ct_t, pc_ct_clean, 5, 30);

% Grid wise donwsample
pc_ct_clean = pcdownsample(pc_ct_clean,'gridAverage',1);