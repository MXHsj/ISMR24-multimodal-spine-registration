% ========================================================================
% file name:    calibrate_clarius_c3hd_phantom_2d.m
% desciption:   us fov calibration using wiring phantom (only run once)
% author:       Xihan Ma
% date:         2023-11-13
% ========================================================================
clc; clear; close all;
addpath('../utils/')

%% load calibration image (labour&delivery mode, 10cm depth)
calib_img_folder = '../dataset/2023-11-13_spine_phantom_franka_US/';
calib_img_bag_name = '2023-11-13-15-32-33_calibrate.bag';
calib_img_bag = rosbag([calib_img_folder, calib_img_bag_name]);
calib_img_msg = readMessages(select(calib_img_bag, 'Topic', 'Clarius/US'));
calib_img = readImage(calib_img_msg{1});

imagesc(calib_img); colormap gray; axis equal tight off;

HEIGHT = size(calib_img, 1);
WIDTH = size(calib_img, 2);

%% calibration target detection
% ===== ROI =====
ROI_X = 320;        % lateral start
ROI_Z = 150;        % axial start
ROI_W = 80;        % lateral width
ROI_H = 65;        % axial height
% ===============
roi = calib_img(ROI_Z:ROI_Z+ROI_H-1, ...
                ROI_X:ROI_X+ROI_W-1);
roi = imnlmfilt(roi, 'DegreeOfSmoothing', 30, ...
                     'SearchWindowSize', 11, ...
                     'ComparisonWindowSize', 11);

[pks,peak_r,peak_c] = peaks2(double(roi),'MinPeakHeight',200);  % find peaks by 8-neighbor search

figure('Position', [1920/5, 1080/5, 1000, 400])
layout = tiledlayout(1,2);

nexttile
imagesc(roi); colormap gray;
hold on
plot(peak_c, peak_r, 'rx', 'MarkerSize', 10)
hold off

nexttile
surf(roi, 'EdgeColor','none')
hold on
plot3(peak_c, peak_r, pks, 'or', 'MarkerSize', 10)

%% calibrate FOV (only works for YT wiring phantom)
NUM_LAT_GR = 2;     % number of lateral groups
NUM_AXI_GR = 3;     % number of axial groups
LAT_INTERV = 10;    % actual lateral interval [mm]
AXI_INTERV = 5;     % actual axial interval [mm]

lat_gr_idx = kmeans(peak_c, NUM_LAT_GR);
axi_gr_idx = kmeans(peak_r, NUM_AXI_GR);

lat_pos = [];
for i = 1:NUM_LAT_GR
    lat_pos = [lat_pos, mean(peak_c(lat_gr_idx == i))];
end

axi_pos = [];
for j = 1:NUM_AXI_GR
    axi_pos = [axi_pos, mean(peak_r(axi_gr_idx == j))];
end

lat_pix_interv = pinv(ones(length(lat_pos)-1, 1))*abs(diff(lat_pos))';
axi_pix_interv = pinv(ones(length(axi_pos)-1, 1))*abs(diff(axi_pos))';

lat_pix_res = LAT_INTERV/lat_pix_interv;    % resolution: mm/pix
axi_pix_res = AXI_INTERV/axi_pix_interv;    % resolution: mm/pix

fprintf('axial resolution: %.4f [mm/pix]\nlateral resolution: %.4f[mm/pix]\n', ...
        axi_pix_res, lat_pix_res);

%% generate calibration summary
summary = table(axi_pix_res, lat_pix_res, ...
                'VariableNames', {'axial res [mm/pix]', 'lateral res [mm/pix]'});
file_out = 'calibration.txt';
fullpath = [calib_img_folder, file_out];
writetable(summary, fullpath)

