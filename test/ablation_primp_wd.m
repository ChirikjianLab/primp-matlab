% Script for ablation study on workspace density adaptation.
%
% Data input from "/data/" folder in ".json" format
%
% Author
%   Sipu Ruan, 2023

close all; clear; clc;
add_paths()

% Name of the dataset
dataset_name = 'panda_arm';
% dataset_name = 'lasa_handwriting/pose_data';

% Robot for execution
robot_execute = {'panda_arm', 'kinovaGen3', 'ur5',...
    'kukaIiwa7', 'abbYumi_right_arm', 'atlas_left_hand',...
    'rethinkBaxter_right_hand', 'robotisOpenManipulator'};
% robot_execute = {'panda_arm', 'kinovaGen3', 'ur5', 'kukaIiwa7'};

demo_type = load_dataset_param(dataset_name, 5:9);

%% Run ablations for each demo type
for i = 1:length(robot_execute)
    for j = 1:length(demo_type)
        run_ablation(robot_execute{i}, dataset_name, demo_type{j});
    end
end

function run_ablation(robot_execute, dataset_name, demo_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of steps for interpolated trajectories
n_step = 50;

% Number of sampled end effector poses
n_state = 1e4;

% Group name: 'SE', 'PCG'
group_name = 'PCG';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/ablation/workspace_density/",...
    robot_execute, "/", demo_type, "/");

mkdir(result_folder);

%% Load demo data and compute mean/covariance
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

% Compute trajectory distribution from demonstrations
filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);
[res_primp.mean, res_primp.covariance] = get_pdf_from_demo(g_demo, group_name);

%% Workspace density of the robot for execution
% Poses of distal end of each link for the manipulator
[pose_ee, mdl_execute, ee_name_execute] = generate_robot_ee_pose(n_state, robot_execute);

% Mean and covariance of the poses of each link
pdf_ee = get_pdf_from_pose_SE(pose_ee);
pdf_ee = pdf_ee{end};

%% Ablation study
clc;
disp('Ablation study for workspace density adaptation in PRIMP')
disp(['Dataset: ', dataset_name])
disp(['Robot for execution: ', robot_execute])
disp(['Demo type: ', demo_type])
disp(['Group: ', group_name])

% Parameters
param.n_sample = 0;
param.group_name = group_name;

res_primp.group_name = param.group_name;
res_primp_fused.group_name = param.group_name;

% PRIMP fused with workspace density of the robot for execution
disp('Fusion with workspace density of robot arm')
primp_fused_obj = PRIMP(res_primp.mean.matrix, res_primp.covariance, param);
[res_primp_fused.mean, res_primp_fused.covariance] =...
    primp_fused_obj.get_fusion_workspace_density(pdf_ee.mean, pdf_ee.cov);

%% Compute Yoshikawa manipulability measure
disp('>> Evaluation: manipulability')
metric_primp.manipulability = compute_manipulability_from_ee_pose(res_primp.mean.matrix, mdl_execute, ee_name_execute);
metric_primp_fused.manipulability = compute_manipulability_from_ee_pose(res_primp_fused.mean, mdl_execute, ee_name_execute);

%% Compute Mahalanobis distance with workspace density
disp('>> Evaluation: Mahalanobis distance with workspace density')
metric_primp.distance = compute_mahalanobis_distance(res_primp.mean.matrix, pdf_ee.mean, pdf_ee.cov, group_name);
metric_primp_fused.distance = compute_mahalanobis_distance(res_primp_fused.mean, pdf_ee.mean, pdf_ee.cov, group_name);

%% Evaluation of ablation study
% Store results as .mat file
res_filename = strcat(result_folder, "result_ablation_primp_wd.mat");
save(res_filename, "res_primp", "res_primp_fused", "metric_primp",...
    "metric_primp_fused", "group_name", "pdf_ee");

% Display and store command window
diary_filename = strcat(result_folder, "result_ablation_primp_wd.txt");
if exist(diary_filename, 'file') ; delete(diary_filename); end
diary(diary_filename);

disp('===============================================================')
disp('Ablation results for workspace density adaptation in PRIMP')
disp(['Group: ', group_name])
disp('===============================================================')

disp('>>>> PRIMP with workspace density adaptation <<<<')
disp('---- Manipulability for mean trajectory ----')
disp(num2str( mean(metric_primp_fused.manipulability, 1) ))

disp('---- Mahalanobis distance to WD for mean trajectory ----')
disp(num2str( mean(metric_primp_fused.distance, 1) ))

disp('---------------------------------------------------------------')

disp('>>>> PRIMP (ablated) <<<<')
disp('---- Manipulability for mean trajectory ----')
disp(num2str( mean(metric_primp.manipulability, 1) ))

disp('---- Mahalanobis distance to WD for mean trajectory ----')
disp(num2str( mean(metric_primp.distance, 1) ))

diary off
end