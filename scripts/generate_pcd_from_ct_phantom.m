% ========================================================================
% file name:    generate_pcd_from_ct_phantom.m
% desciption:   generate pointcloud from ct spine phantom
% author:       Xihan Ma
% date:         2023-11-09
% ========================================================================
clc; clear; close all;

% ====================================================
IS_WRITE_PCD = false;   % set to true to save generated pointcloud
% ====================================================

file_path = fileparts(matlab.desktop.editor.getActiveFilename);
cd(file_path)
addpath('../utils/')

%% load dicom
ct_folder = '../dataset/2023-10-26_spine_phantom_CT/';
dcm_folder = 'AX_BONE_0005/';
tic;
[ct_volume, spatial, ~] = dicomreadVolume([ct_folder, dcm_folder]);
ct_volume = squeeze(ct_volume);     % remove singleton dimensions
num_frms = size(ct_volume, 3);
axi_pix_res = spatial.PixelSpacings(1,1);                   % [mm/pix]
lat_pix_res = spatial.PixelSpacings(1,2);                   % [mm/pix]
ele_pix_res = mean(diff(spatial.PatientPositions(:,3)));    % [mm/pix]
fprintf('read dicom files took %.3f sec.\n', toc);

%% generate vertebrae pointcloud
% ========== params ==========
VERT_THRESH = 1500;     % intensity thresholding
DWNSAMP_FACTOR = 0.5;   % pointcloud downsample ratio
VERT_ROI_X = 135;
VERT_ROI_Z = 20;
VERT_ROI_H = 190;
VERT_ROI_W = 245;
% ============================

% extract vertebrae & write to pointcloud
vertebrae_volume = ct_volume(VERT_ROI_Z : VERT_ROI_Z+VERT_ROI_H-1, ...
                             VERT_ROI_X : VERT_ROI_X+VERT_ROI_W-1, ...
                             :);
vet_x = []; % lateral [mm]
vet_y = []; % elevational [mm]
vet_z = []; % axial [mm]
vet_x_int = []; vet_y_int = []; vet_z_int = [];    % preserve intensity values
tic;
for frm = 1:num_frms
    fprintf('process (%d/%d)-th slice ... \n', frm, num_frms);
    frame = vertebrae_volume(:,:,frm);
    [row, col] = find(frame > VERT_THRESH);
    if ~isempty(row) && ~isempty(col)
        x = lat_pix_res.*(col-1);
        y = ele_pix_res.*(frm-1)*ones(length(col), 1);
        z = -axi_pix_res.*(row-1);
    
        xint = zeros(1,length(row),'uint16');
        yint = zeros(1,length(row),'uint16');
        zint = zeros(1,length(row),'uint16');
        for i = 1:length(row)
            xint(i) = frame(row(i),col(i));
            yint(i) = frame(row(i),col(i));
            zint(i) = frame(row(i),col(i));
        end

        vet_x = cat(2, vet_x, x');
        vet_y = cat(2, vet_y, y');
        vet_z = cat(2, vet_z, z');
        vet_x_int = cat(2, vet_x_int, xint);
        vet_y_int = cat(2, vet_y_int, yint);
        vet_z_int = cat(2, vet_z_int, zint);
    end 
end

vet_xyz = [vet_x'-mean(vet_x), vet_y'-mean(vet_y), vet_z'-mean(vet_z)];
vet_int = [vet_x_int; vet_y_int; vet_z_int]';                 % intensity
% pntcloud = pointCloud(vet_xyz, 'Color', vet_int);
vet_cloud = pointCloud(vet_xyz);
vet_cloud = pcdenoise(vet_cloud);                             % denoise
vet_cloud = pcdownsample(vet_cloud, 'random', DWNSAMP_FACTOR);
fprintf('generate pointcloud took %.3f sec.\n', toc);

if IS_WRITE_PCD
    full_name = [ct_folder, 'vertebrae.pcd'];
    pcwrite(vet_cloud, full_name);
    fprintf('pointcloud saved to %s\n', full_name);
end

%% generate fiducial pointcloud
% ========== params ==========
FID_COLOR = zeros(6, 3); FID_COLOR(:, 1) = 1;   % color
% ============================

% ========== fiducial positions [mm] ==========
% ***** fiducials @ L1 *****
FID_L1_L_X = (66-VERT_ROI_X-1) * lat_pix_res;
FID_L1_L_Z = -(167-VERT_ROI_Z-1) * axi_pix_res;
FID_L1_L_Y = (97-1) * ele_pix_res;    % slice idx

FID_L1_R_X = (436-VERT_ROI_X-1) * lat_pix_res;
FID_L1_R_Z = -(178-VERT_ROI_Z-1) * axi_pix_res;
FID_L1_R_Y = (98-1) * ele_pix_res;

% ***** fiducials @ L2 *****
FID_L2_L_X = (68-VERT_ROI_X-1) * lat_pix_res;
FID_L2_L_Z = -(169-VERT_ROI_Z-1) * axi_pix_res;
FID_L2_L_Y = (69-1) * ele_pix_res;    % 68?

FID_L2_R_X = (439-VERT_ROI_X-1) * lat_pix_res;
FID_L2_R_Z = -(181-VERT_ROI_Z-1) * axi_pix_res;
FID_L2_R_Y = (69-1) * ele_pix_res;

% ***** fiducials @ L3 *****
FID_L3_L_X = (70-VERT_ROI_X-1) * lat_pix_res;
FID_L3_L_Z = -(169-VERT_ROI_Z-1) * axi_pix_res;
FID_L3_L_Y = (46-1) * ele_pix_res;

FID_L3_R_X = (439-VERT_ROI_X-1) * lat_pix_res;
FID_L3_R_Z = -(179-VERT_ROI_Z-1) * axi_pix_res;
FID_L3_R_Y = (46-1) * ele_pix_res;
% =============================================

fid_xyz = [FID_L1_L_X, FID_L1_L_Y, FID_L1_L_Z; ...
           FID_L1_R_X, FID_L1_R_Y, FID_L1_R_Z; ...
           FID_L2_L_X, FID_L2_L_Y, FID_L2_L_Z; ...
           FID_L2_R_X, FID_L2_R_Y, FID_L2_R_Z; ...
           FID_L3_L_X, FID_L3_L_Y, FID_L3_L_Z; ...
           FID_L3_R_X, FID_L3_R_Y, FID_L3_R_Z];
fid_xyz(:, 1) = fid_xyz(:, 1) - mean(fid_xyz(:, 1));
fid_xyz(:, 2) = fid_xyz(:, 2) - mean(fid_xyz(:, 2));
fid_xyz(:, 3) = fid_xyz(:, 3) - mean(fid_xyz(:, 3));
fid_cloud = pointCloud(fid_xyz, 'Color', FID_COLOR);

if IS_WRITE_PCD
    full_name = [ct_folder, 'fiducials.pcd'];
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
    'ZColor',ZAX_COLOR);
view(3)
