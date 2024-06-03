% ========================================================================
% file name:    generate_pcd_from_us_phantom.m
% desciption:   generate surface pointcloud from robotic us spine phantom
% author:       Xihan Ma
% date:         2023-11-13
% ========================================================================
clc; clear; close all;

% ====================================================
IS_WRITE_PCD = false;   % set to true to save generated pointcloud
% ====================================================

file_path = fileparts(matlab.desktop.editor.getActiveFilename);
cd(file_path)
addpath('../utils/');

%% load ROS messages (i.e., US - pose pairs)
tic;
us_folder = '../dataset/2023-11-13_spine_phantom_franka_US/';
bag_name = '2023-11-13-15-24-09.bag';
calib_file_name = 'calibration.txt';

franka_us_bag = rosbag([us_folder, bag_name]);
vert_time = franka_us_bag.StartTime + [44, 78.5];
fidL1_time = franka_us_bag.StartTime + [15.6, 17.6];
fidL2_time = franka_us_bag.StartTime + [22.4, 24.7];
fidL3_time = franka_us_bag.StartTime + [31.0, 33.2];
fidR1_time = franka_us_bag.StartTime + [94.7, 96.7];
fidR2_time = franka_us_bag.StartTime + [102.0, 103.8];
fidR3_time = franka_us_bag.StartTime + [110.0, 112.4];

vert_img_bag = select(franka_us_bag, 'Time', vert_time, 'Topic', 'Clarius/US');
vert_pos_bag = select(franka_us_bag, 'Time', vert_time, 'Topic', 'franka_state_custom');
vert_img_msg = readMessages(vert_img_bag);
vert_pos_msg = findSyncPos(vert_img_bag, vert_pos_bag);

vert_img = msg2frms(vert_img_msg);
vert_pos = msg2pose(vert_pos_msg);

[fidL1_img, fidL1_pos, fidL2_img, fidL2_pos, fidL3_img, fidL3_pos, ...
 fidR1_img, fidR1_pos, fidR2_img, fidR2_pos, fidR3_img, fidR3_pos] = ...
  getFiducials(franka_us_bag, ...
               fidL1_time, fidL2_time, fidL3_time, ...
               fidR1_time, fidR2_time, fidR3_time);

warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames')
us_calib = readtable([us_folder, calib_file_name], 'VariableNamingRule', 'modify');
axi_pix_res = us_calib.axialRes_mm_pix_;        % [mm/pix]
lat_pix_res = us_calib.lateralRes_mm_pix_;      % [mm/pix]
fprintf('load data took %.3f sec.\n', toc);

%% generate vertebrae pointcloud
% ========== params ==========
VERT_THRESH = 50;           % intensity thresholding
SLICE_DWNSAMP_FACTOR = 3;   % elevational downsample ratio
PC_DWNSAMP_FACTOR = 0.1;    % pointcloud uniform downsample ratio
IS_SMOOTH_TRAJ = true;      % if smooth robot scan trajectory
VERT_ROI_X = 140;
VERT_ROI_Z = 80;
VERT_ROI_H = 180;
VERT_ROI_W = 340;
% ============================

% extract vertebrae & write to pointcloud
vertebrae_volume = vert_img(VERT_ROI_Z : VERT_ROI_Z+VERT_ROI_H-1, ...
                            VERT_ROI_X : VERT_ROI_X+VERT_ROI_W-1, ...
                            :);

if IS_SMOOTH_TRAJ
    vert_pos = smoothTraj(vert_pos);
end

vet_x = []; % lateral [mm]
vet_y = []; % elevational [mm]
vet_z = []; % axial [mm]
vet_x_int = []; vet_y_int = []; vet_z_int = [];    % preserve intensity values
tic;
for frm = 1:size(vert_img, 3)
    fprintf('process (%d/%d)-th slice ... \n', frm, size(vert_img, 3));
    pos_interv = round(size(vert_pos,3)/size(vert_img, 3));
%     frame = vertebrae_volume(:,:,frm);
%     [row, col] = find(frame > VERT_THRESH);
    frame = vert_img(:,:,frm);
    [mask, ~] = segmentBoneSurface(frame, 'c3hd', false);
    [row, col] = find(mask == true);
    if ~isempty(row) && ~isempty(col)
        xlocal = lat_pix_res.*(col+VERT_ROI_X-1);
        zlocal = axi_pix_res.*(row+VERT_ROI_Z-1);
        ylocal = zeros(length(col), 1); % use tracked robot pose
        
        T = vert_pos(:,:,frm);
        [x, y, z] = transformPoints(T, xlocal, ylocal, zlocal);
        
        xint = zeros(1,length(row),'uint8');
        yint = zeros(1,length(row),'uint8');
        zint = zeros(1,length(row),'uint8');
        for i = 1:length(row)
            xint(i) = frame(row(i),col(i));
            yint(i) = frame(row(i),col(i));
            zint(i) = frame(row(i),col(i));
        end

        vet_x = cat(2, vet_x, x');
        vet_y = cat(2, vet_y, -y');
        vet_z = cat(2, vet_z, z');
        vet_x_int = cat(2, vet_x_int, xint);
        vet_y_int = cat(2, vet_y_int, yint);
        vet_z_int = cat(2, vet_z_int, zint);
    end 
end

% normalize around centroid
vet_xyz = [vet_x'-mean(vet_x), vet_y'-mean(vet_y), vet_z'-mean(vet_z)];
vet_int = [vet_x_int; vet_y_int; vet_z_int]';   % intensity

% pntcloud = pointCloud(vet_xyz, 'Color', vet_int);
vet_cloud = pointCloud(vet_xyz);
vet_cloud = pcdenoise(vet_cloud, 'PreserveStructure', true, ...
                                 'Threshold', 0.3);
vet_cloud = pcdownsample(vet_cloud, 'random', PC_DWNSAMP_FACTOR);
fprintf('generate pointcloud took %.3f sec.\n', toc);

if IS_WRITE_PCD
    full_name = [us_folder, 'vertebrae.pcd'];
    pcwrite(vet_cloud, full_name);
    fprintf('pointcloud saved to %s\n', full_name);
end

%% generate fiducial pointcloud
% ========== params ==========
FID_COLOR = zeros(6, 3); FID_COLOR(:, 1) = 1;   % color
IS_SMOOTH_TRAJ = true;      % if smooth robot scan trajectory
% ============================

if IS_SMOOTH_TRAJ
    fidL1_pos = smoothTraj(fidL1_pos);
    fidR1_pos = smoothTraj(fidR1_pos);
    fidL2_pos = smoothTraj(fidL2_pos);
    fidR2_pos = smoothTraj(fidR2_pos);
    fidL3_pos = smoothTraj(fidL3_pos);
    fidR3_pos = smoothTraj(fidR3_pos);
end

% ========== fiducial positions [mm] ==========
% ***** fiducials @ L1 *****
T_L1_L = fidL1_pos(:, :, 25);
[FID_L1_L_X, FID_L1_L_Y, FID_L1_L_Z] = transformPoints(T_L1_L, ...
                                        334 * lat_pix_res, ...
                                        0, ...
                                        221 * axi_pix_res);

T_L1_R = fidR1_pos(:,:,25);
[FID_L1_R_X, FID_L1_R_Y, FID_L1_R_Z] = transformPoints(T_L1_R, ...
                                        297 * lat_pix_res, ...
                                        0, ...
                                        214 * axi_pix_res);

% ***** fiducials @ L2 *****
T_L2_L = fidL2_pos(:, :, 26);
[FID_L2_L_X, FID_L2_L_Y, FID_L2_L_Z] = transformPoints(T_L2_L, ...
                                        331 * lat_pix_res, ...
                                        0, ...
                                        217 * axi_pix_res);

T_L2_R = fidR2_pos(:, :, 18);
[FID_L2_R_X, FID_L2_R_Y, FID_L2_R_Z] = transformPoints(T_L2_R, ...
                                        293 * lat_pix_res, ...
                                        0, ...
                                        216 * axi_pix_res);

% ***** fiducials @ L3 *****
T_L3_L = fidL3_pos(:, :, 25);
[FID_L3_L_X, FID_L3_L_Y, FID_L3_L_Z] = transformPoints(T_L3_L, ...
                                        331 * lat_pix_res, ...
                                        0, ...
                                        226 * axi_pix_res);

T_L3_R = fidR3_pos(:, :, 25);
[FID_L3_R_X, FID_L3_R_Y, FID_L3_R_Z] = transformPoints(T_L3_R, ...
                                        295 * lat_pix_res, ...
                                        0, ...
                                        226 * axi_pix_res);
% =============================================

fid_xyz = [-FID_L3_L_X, -FID_L3_L_Y, FID_L3_L_Z; ...
           -FID_L3_R_X, -FID_L3_R_Y, FID_L3_R_Z; ...
           -FID_L2_L_X, -FID_L2_L_Y, FID_L2_L_Z; ...
           -FID_L2_R_X, -FID_L2_R_Y, FID_L2_R_Z; ...
           -FID_L1_L_X, -FID_L1_L_Y, FID_L1_L_Z; ...
           -FID_L1_R_X, -FID_L1_R_Y, FID_L1_R_Z];
fid_xyz(:, 1) = fid_xyz(:, 1) - mean(fid_xyz(:, 1));
fid_xyz(:, 2) = fid_xyz(:, 2) - mean(fid_xyz(:, 2));
fid_xyz(:, 3) = fid_xyz(:, 3) - mean(fid_xyz(:, 3));
fid_cloud = pointCloud(fid_xyz, 'Color', FID_COLOR);

if IS_WRITE_PCD   
    full_name = [us_folder, 'fiducials.pcd'];
    pcwrite(fid_cloud, full_name);
    fprintf('pointcloud saved to %s\n', full_name);
end

%% vis vertebrae & fiducial pointcloud
% ========== vis params ==========
WIN_WIDTH = 800;
WIN_HEIGHT = 500;
XAX_COLOR = [0.15 0.15 0.15];
YAX_COLOR = [0.15 0.15 0.15];
ZAX_COLOR = [0.15 0.15 0.15];
PNT_SIZE = 3;
FID_SIZE = 200;
% ================================

figure('Position', [1920/5, 1080/5, WIN_WIDTH, WIN_HEIGHT])
pcshow(vet_cloud, 'MarkerSize', PNT_SIZE)
hold on
pcshow(fid_cloud, 'MarkerSize', FID_SIZE)
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')
axis equal tight off
set(gcf, 'color', 'w'); % make background white
set(gca, 'color', 'w', ...
    'XColor', XAX_COLOR, ...
    'YColor', YAX_COLOR, ...
    'ZColor', ZAX_COLOR);
view(3)

%% utilities
% *******************************************
% read frames from ROS Image message
% *******************************************
function [frms] = msg2frms(msg)
    frms = zeros(msg{1}.Height, msg{1}.Width, length(msg), 'uint8');
    for frm = 1:size(frms, 3)
        frms(:,:,frm) = readImage(msg{frm});
    end
end

% *******************************************
% read homogeneous transformations from ROS Float64MultiArray message
% *******************************************
function [pose] = msg2pose(msg)
    pose = zeros(4, 4, length(msg));
    for frm = 1:size(pose, 3)
        pose(:, :, frm) = reshape(msg{frm}.Data, 4, 4)';
        pose(1:3, end, frm) = 1e3*pose(1:3, end, frm); % [m] -> [mm]
    end
end

% *******************************************
% find synchronized probe pose for each US image
% *******************************************
function [pos_msg] = findSyncPos(img_bag, pos_bag)
%     tic;
    num_frm = size(img_bag.MessageList, 1);
    interval = diff(img_bag.MessageList.Time);
    pos_msg = cell(num_frm, 1);
    start_time = img_bag.StartTime;
    for frm = 1:num_frm
        if frm ~= num_frm
            end_time = start_time + interval(frm);
        else
            end_time = start_time + mean(interval);
        end
        img_stamp = img_bag.MessageList.Time(frm);
        pos_bag_tmp = select(pos_bag, 'Time', [start_time, end_time], ...
                             'Topic', 'franka_state_custom');
        timestamp_diff = abs(img_stamp - pos_bag_tmp.MessageList.Time);
        [~, pos_stamp_idx] = min(timestamp_diff);   % find nearest timestamp
        pos_msg_queue = readMessages(pos_bag_tmp);
        pos_msg{frm} = pos_msg_queue{pos_stamp_idx};
        start_time = end_time;
        fprintf('frame: %d/%d\tpose queue size: %d\tnearest pose idx:%d\n', ...
                frm, num_frm, length(pos_msg_queue), pos_stamp_idx)
    end
%     fprintf('find sychronized pose took %.3f sec\n', toc);
end

% *******************************************
% get fiducial marker frames
% *******************************************
function [fidL1_img, fidL1_pos, fidL2_img, fidL2_pos, fidL3_img, fidL3_pos, ...
          fidR1_img, fidR1_pos, fidR2_img, fidR2_pos, fidR3_img, fidR3_pos] = ...
          getFiducials(us_bag, ...
                       fidL1_time, fidL2_time, fidL3_time, ...
                       fidR1_time, fidR2_time, fidR3_time)

% L1
fidL1_img_bag = select(us_bag, 'Time', fidL1_time, 'Topic', 'Clarius/US');
fidL1_pos_bag = select(us_bag, 'Time', fidL1_time, 'Topic', 'franka_state_custom');
fidL1_img_msg = readMessages(fidL1_img_bag);
fidL1_pos_msg = findSyncPos(fidL1_img_bag, fidL1_pos_bag);
fidL1_img = msg2frms(fidL1_img_msg);
fidL1_pos = msg2pose(fidL1_pos_msg);
% L2
fidL2_img_bag = select(us_bag, 'Time', fidL2_time, 'Topic', 'Clarius/US');
fidL2_pos_bag = select(us_bag, 'Time', fidL2_time, 'Topic', 'franka_state_custom');
fidL2_img_msg = readMessages(fidL2_img_bag);
fidL2_pos_msg = findSyncPos(fidL2_img_bag, fidL2_pos_bag);
fidL2_img = msg2frms(fidL2_img_msg);
fidL2_pos = msg2pose(fidL2_pos_msg);
% L3
fidL3_img_bag = select(us_bag, 'Time', fidL3_time, 'Topic', 'Clarius/US');
fidL3_pos_bag = select(us_bag, 'Time', fidL3_time, 'Topic', 'franka_state_custom');
fidL3_img_msg = readMessages(fidL3_img_bag);
fidL3_pos_msg = findSyncPos(fidL3_img_bag, fidL3_pos_bag);
fidL3_img = msg2frms(fidL3_img_msg);
fidL3_pos = msg2pose(fidL3_pos_msg);
% R1
fidR1_img_bag = select(us_bag, 'Time', fidR1_time, 'Topic', 'Clarius/US');
fidR1_pos_bag = select(us_bag, 'Time', fidR1_time, 'Topic', 'franka_state_custom');
fidR1_img_msg = readMessages(fidR1_img_bag);
fidR1_pos_msg = findSyncPos(fidR1_img_bag, fidR1_pos_bag);
fidR1_img = msg2frms(fidR1_img_msg);
fidR1_pos = msg2pose(fidR1_pos_msg);
% R2
fidR2_img_bag = select(us_bag, 'Time', fidR2_time, 'Topic', 'Clarius/US');
fidR2_pos_bag = select(us_bag, 'Time', fidR2_time, 'Topic', 'franka_state_custom');
fidR2_img_msg = readMessages(fidR2_img_bag);
fidR2_pos_msg = findSyncPos(fidR2_img_bag, fidR2_pos_bag);
fidR2_img = msg2frms(fidR2_img_msg);
fidR2_pos = msg2pose(fidR2_pos_msg);
% R3
fidR3_img_bag = select(us_bag, 'Time', fidR3_time, 'Topic', 'Clarius/US');
fidR3_pos_bag = select(us_bag, 'Time', fidR3_time, 'Topic', 'franka_state_custom');
fidR3_img_msg = readMessages(fidR3_img_bag);
fidR3_pos_msg = findSyncPos(fidR3_img_bag, fidR3_pos_bag);
fidR3_img = msg2frms(fidR3_img_msg);
fidR3_pos = msg2pose(fidR3_pos_msg);

end

% *******************************************
% smooth robot scan trajectory
% *******************************************
function [traj_] = smoothTraj(traj)
    x_traj = abs(traj(1,end,end) - traj(1,end,1));
    y_traj = abs(traj(2,end,end) - traj(2,end,1));
    [~, principle_axis_idx] = max([x_traj, y_traj]);
    traj_ = traj;
    traj_(principle_axis_idx,end,:) = linspace(traj(principle_axis_idx,end,1), ...
                                                  traj(principle_axis_idx,end,end), ...
                                                  size(traj, 3));
end

