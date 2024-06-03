% ========================================================================
% file name:    registration_ct_us.m
% desciption:   register US & CT spine phantom using ICP & CPD
% author:       Xiao Zhang, Xihan Ma
% date:         2023-11-20
% ========================================================================
clc; clear; close all
file_path = fileparts(matlab.desktop.editor.getActiveFilename);
cd(file_path)
addpath('../../utils/')

%% Data loading
tic;
ct_folder = '..\dataset\2023-10-26_spine_phantom_CT\';
us_folder = '..\dataset\2023-11-13_spine_phantom_franka_US\';
pc_ct_raw = pcread([ct_folder, 'vertebrae.pcd']);
pc_us_raw = pcread([us_folder, 'vertebrae.pcd']);
fid_ct_raw = pcread([ct_folder, 'fiducials.pcd']);
fid_us_raw = pcread([us_folder, 'fiducials.pcd']);
fprintf('load pointcloud data took %.3f sec\n', toc)

% % ========== raw input ==========
% figure('Name', 'Vertbrae CT Scan Raw');
% showTissueFiducialPcd(pc_ct_raw, fid_ct_raw)
% 
% figure('Name', 'Vertbrae US Scan Raw')
% showTissueFiducialPcd(pc_us_raw, fid_us_raw)
% % ===============================

%% Pre-process 
IS_WRITE_PCD = false;
% global fid_us_raw
% ===== denoise + surface extraction =====
tic;
pc_ct_ = preprocess_ct_pcd(pc_ct_raw);
pc_us_ = preprocess_us_pcd(pc_us_raw);

% ===== find US X-Y scaling =====
[tform_fid, fid_ct_mv_fid] = pcregistercpd(fid_ct_raw, fid_us_raw, ...
                                           'Transform', 'Affine', 'OutlierRatio', 0);
rmse_rescale = helperCalculatePointCloudRMSE(fid_ct_mv_fid, fid_us_raw);

T_aff = tform_fid.A;
sx = 1.074/sqrt(T_aff(1,1)^2 + T_aff(2,1)^2 + T_aff(3,1)^2);
sy = 1.00/sqrt(T_aff(1,2)^2 + T_aff(2,2)^2 + T_aff(3,2)^2);
sz = 1.00/sqrt(T_aff(1,3)^2 + T_aff(2,2)^2 + T_aff(3,3)^2);

fprintf('US X scaling: %.3f, Y scaling: %.3f, rescaled fiducial rmse: %.3f\n', ...
        sx, sy, rmse_rescale)
clear tform_fid fid_ct_mv_fid

[pc_us_clean] = scale_pcd_c3hd(pc_us_, sx, sy, 1.333);
[fid_us] = scale_pcd_c3hd(fid_us_raw, sx, sy, sz);
pc_ct_clean = pc_ct_;
fid_ct = fid_ct_raw;
fprintf('preprocess US CT pointcloud took %.3f sec\n', toc)

if IS_WRITE_PCD
    pcwrite(pc_ct_clean, [ct_folder, 'vertebrae_clean.pcd']);
    pcwrite(pc_us_clean, [us_folder, 'vertebrae_clean_scaled.pcd']);
    pcwrite(fid_us, [us_folder, 'fiducials_scaled.pcd']);
    fprintf('processed pointcloud saved\n');
end

% ========== surface extracted ==========
figure();
showTissueFiducialPcd(pc_ct_clean, fid_ct)
title('Vertbrae CT Surface Extraction')

figure();
showTissueFiducialPcd(pc_us_clean, fid_us)
title('Vertbrae US Scan Surface Extraction')
% =======================================

%% Fiducial pointcloud registration
% % ===== method1: close form solution =====
% [T, rmse_fid] = cloud2cloud(fid_ct.Location', fid_us.Location');
% [fid_ct_mv_x, fid_ct_mv_y, fid_ct_mv_z] = transformPoints(T, fid_ct.Location(:,1), fid_ct.Location(:,2), fid_ct.Location(:,3));
% fid_ct_mv_fid = pointCloud([fid_ct_mv_x, fid_ct_mv_y, fid_ct_mv_z], 'Color', fid_ct.Color);
% [pc_ct_clean_mv_x, pc_ct_clean_mv_y, pc_ct_clean_mv_z] = transformPoints(T, pc_ct_clean.Location(:,1), pc_ct_clean.Location(:,2), pc_ct_clean.Location(:,3));
% pc_ct_clean_mv_fid = pointCloud([pc_ct_clean_mv_x, pc_ct_clean_mv_y, pc_ct_clean_mv_z]);

% ===== method2: ICP =====
[tform_fid, fid_ct_mv_fid] = pcregistericp(fid_ct, fid_us);
pc_ct_clean_mv_fid = pctransform(pc_ct_clean, tform_fid);
rmse_fid = helperCalculatePointCloudRMSE(fid_ct_mv_fid, fid_us);

figure();
showTissueFiducialPcdPairs(pc_us_clean, fid_us, pc_ct_clean_mv_fid, fid_ct_mv_fid, true)
title(['CT-US fiducial registration', ' RMSE: ', num2str(rmse_fid)])


%% Spine pointcloud registration
% Method FPFH feature based CPD
tic
[tform_cpdfpfh,pc_ct_clean_mv_cpdfpfh, res_cpdfpfh] = helperCPDFPFH(pc_ct_clean, pc_us_clean, 'rigid');
fid_ct_mv_cpdfpfh = pctransform(fid_ct,tform_cpdfpfh);
t_cpd_fpfh = toc;
fprintf('CPD + FPFH registration took %.3f sec\n', t_cpd_fpfh)

% Method CPD
tic
[tform_cpd,pc_ct_clean_mv_cpd, res_cpd] = pcregistercpd(pc_ct_clean, pc_us_clean, ...
                                                        'Transform', 'Rigid', 'OutlierRatio', 0.8);
fid_ct_mv_cpd = pctransform(fid_ct,tform_cpd);
t_cpd = toc;
fprintf('CPD registration took %.3f sec\n', t_cpd)

% Method ICP
tic
[tform_icp,pc_ct_clean_mv_icp, res_icp] = pcregistericp(pc_ct_clean, pc_us_clean, ...
                                                        Metric="pointToPoint");
fid_ct_mv_icp = pctransform(fid_ct, tform_icp);
t_icp = toc;
fprintf('ICP registration took %.3f sec\n', t_icp)

% RMSE Evaluation
rmse_cpdfpfh = helperCalculatePointCloudRMSE(fid_us, fid_ct_mv_cpdfpfh);
rmse_cpd = helperCalculatePointCloudRMSE(fid_us, fid_ct_mv_cpd);
rmse_icp = helperCalculatePointCloudRMSE(fid_us, fid_ct_mv_icp);

%% vis registration results
% ========== CPD FPFH register ==========
figure();
showTissueFiducialPcdPairs(pc_us_clean, fid_us, pc_ct_clean_mv_cpdfpfh, fid_ct_mv_cpdfpfh, true)
title(['CT-US (CPD+FPFH)', 'RMSE: ', num2str(rmse_cpdfpfh), ' Excution Time: ' num2str(t_cpd_fpfh)])
% =======================================

% ========== CPD register ==========
figure();
showTissueFiducialPcdPairs(pc_us_clean, fid_us, pc_ct_clean_mv_cpd, fid_ct_mv_cpd, true)
title(['CT-US (CPD)', 'RMSE: ', num2str(rmse_cpd), ' Excution Time: ', num2str(t_cpd)])
% ==================================

% ========== ICP register ==========
figure()
showTissueFiducialPcdPairs(pc_us_clean, fid_us, pc_ct_clean_mv_icp, fid_ct_mv_icp, true)
title(['CT-US (ICP)', 'RMSE: ', num2str(rmse_icp), ' Excution Time: ', num2str(t_icp)])
% ==================================

%% set randome initial conditions
rng('default')
num_trials = 10;
rotationAngles = rad2deg((pi/3)*rand(num_trials, 3));
translations = 100*rand(num_trials, 3);

rmse_rec = zeros(num_trials, 3); % FRE col1: CPD+fpfh; col2: CPD; col3: ICP
res_rec = zeros(num_trials, 3); % residual err col1: CPD+fpfh; col2: CPD; col3: ICP
exe_time_rec = zeros(num_trials, 3); % exe. time col1: CPD+fpfh; col2: CPD; col3: ICP
for t = 1:num_trials
    fprintf('trial: %d/%d\n', t, num_trials)
    if t > 1
        tform_ = rigidtform3d(rotationAngles(t, :), translations(t, :));
        pc_ct_tmp = pctransform(pc_ct_clean, tform_);
        fid_ct_tmp = pctransform(fid_ct, tform_);
    else    % start with no random transformation
        pc_ct_tmp = pc_ct_clean;
        fid_ct_tmp = fid_ct;
    end
    % Method FPFH feature based CPD
    tic
    [tform_cpdfpfh_, pc_ct_clean_mv_cpdfpfh_, res_rec(t,1)] = helperCPDFPFH(pc_ct_tmp, pc_us_clean, 'rigid');
    fid_ct_mv_cpdfpfh_ = pctransform(fid_ct_tmp, tform_cpdfpfh_);
    exe_time_rec(t, 1) = toc;
    fprintf('CPD + FPFH registration took %.3f sec\n', exe_time_rec(t, 1))
    
    % Method CPD
    tic
    [tform_cpd_, pc_ct_clean_mv_cpd_, res_rec(t,2)] = pcregistercpd(pc_ct_tmp, pc_us_clean, ...
                                                      'Transform', 'Rigid', 'OutlierRatio', 0.8);
    fid_ct_mv_cpd_ = pctransform(fid_ct_tmp, tform_cpd_);
    exe_time_rec(t, 2) = toc;
    fprintf('CPD registration took %.3f sec\n', exe_time_rec(t, 2))
    
    % Method ICP
    tic
    [tform_icp_, pc_ct_clean_mv_icp_, res_rec(t,3)] = pcregistericp(pc_ct_tmp, pc_us_clean);
    fid_ct_mv_icp_ = pctransform(fid_ct_tmp, tform_icp_);
    exe_time_rec(t, 3) = toc;
    fprintf('ICP registration took %.3f sec\n', exe_time_rec(t, 3))

    rmse_rec(t, 1) = helperCalculatePointCloudRMSE(fid_us, fid_ct_mv_cpdfpfh_);
    rmse_rec(t, 2) = helperCalculatePointCloudRMSE(fid_us, fid_ct_mv_cpd_);
    rmse_rec(t, 3) = helperCalculatePointCloudRMSE(fid_us, fid_ct_mv_icp_);
    
    fprintf('FRE:\nCPD+fpfh: %.3f\tCPD: %.3f\tICP: %.3f\n', ...
            rmse_rec(t, 1), rmse_rec(t, 2), rmse_rec(t, 3))
end

save('errors_rec.mat', 'rmse_rec', 'res_rec', 'exe_time_rec')

%% initial condition test result
if ~exist('rmse_rec', 'var')
    load('errors_rec.mat')
end
% ========== FRE ==========
figure('Position',[1920/3, 1080/3, 550, 400])
boxchart(rmse_rec,'BoxFaceColor','#77AC30'); box on; hold on; 
xticklabels({'CPD+FPFH', 'CPD', 'ICP'})
ylabel('FRE [mm]')
plot(rmse_rec', '.', 'MarkerSize', 12, 'Color', [48,55,61]./255)

% ========== residual RMSE ==========
figure('Position',[1920/3, 1080/3, 550, 400])
boxchart(res_rec,'BoxFaceColor','#77AC30'); box on; hold on;
xticklabels({'CPD+FPFH', 'CPD', 'ICP'})
ylabel('residual RMSE [mm]')
plot(res_rec', '.', 'MarkerSize', 12, 'Color', [48,55,61]./255)

% ========== exe. time ==========
figure('Position',[1920/3, 1080/3, 550, 400])
boxchart(exe_time_rec,'BoxFaceColor','#77AC30'); box on; hold on; 
xticklabels({'CPD+FPFH', 'CPD', 'ICP'})
ylabel('time [sec]')
plot(exe_time_rec', '.', 'MarkerSize', 12, 'Color', [48,55,61]./255)

% ========== linear correlation between FRE & residual RMSE ==========
fprintf('FPFH+CPD fre vs. res corr: %.3f\n', corr(res_rec(:,1), rmse_rec(:,1)))
fprintf('CPD fre vs. res corr: %.3f\n', corr(res_rec(:,2), rmse_rec(:,2)))
fprintf('ICP fre vs. res corr: %.3f\n', corr(res_rec(:,3), rmse_rec(:,3)))

% p_cpd_fpfh = polyfit(res_rec(:,1), rmse_rec(:,1), 1);
% p_cpd = polyfit(res_rec(:,2), rmse_rec(:,2), 1);
% p_icp = polyfit(res_rec(:,3), rmse_rec(:,3), 1);

faceColors = [41,134,204; ...
              248,148,0; ...
              159,98,170]./255;
figure('Position',[1920/3, 1080/3, 550, 400])
hold on
splt = scatter(res_rec, rmse_rec, 40, 'filled');
colororder(faceColors)
% plot(res_rec(:, 1), polyval(p_cpd_fpfh, res_rec(:, 1)))
% plot(res_rec(:, 2), polyval(p_cpd, res_rec(:, 2)))
% plot(res_rec(:, 3), polyval(p_icp, res_rec(:, 3)))
hold off
box on;
legend('FPFH+CPD', 'CPD', 'ICP', 'Location', 'northwest')
xlabel('residual RMSE [mm]'); ylabel('FRE [mm]')
