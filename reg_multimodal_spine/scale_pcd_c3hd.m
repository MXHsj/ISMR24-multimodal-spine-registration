% ========================================================================
% file name:    scale_pcd_c3hd.m
% desciption:   scale pcd to compensate for axial pixel res. difference
% author:       Xihan Ma
% date:         2023-11-17
% ========================================================================
function [pc_us_scaled] = scale_pcd_c3hd(pc_us, sx, sy, sz)

% ========== params ==========
X_SCALE_FACTOR = sx;
Y_SCALE_FACTOR = sy;
Z_SCALE_FACTOR = sz;
% ============================

x_ = pc_us.Location(:, 1) * X_SCALE_FACTOR;
y_ = pc_us.Location(:, 2) * Y_SCALE_FACTOR;
z_ = pc_us.Location(:, 3) * Z_SCALE_FACTOR;

pc_us_scaled = pointCloud([x_, y_, z_], "Color", pc_us.Color);

