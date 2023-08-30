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
robot_execute = ['panda_arm', 'kinovaGen3', 'universalUR10',...
    'kukaIiwa7', 'abbYumi_right_arm', 'atlas_left_hand',...
    'rethinkBaxter_right_hand', 'robotisOpenManipulator'];

demo_type = load_dataset_param(dataset_name);

%% Run ablations for each demo type
for i = 1:length(robot_execute)
    for j = 1:length(demo_type)
        run_ablation(robot_execute(i), dataset_name, demo_type{j});
    end
end

function run_ablation(robot_execute, dataset_name, demo_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of steps for interpolated trajectories
n_step = 50;

% Number of sampled end effector poses
n_state = 1000;

% Group name: 'SE', 'PCG'
group_name = 'SE';
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
res_primp_fused.pdf_ee = get_pdf_from_pose_SE(pose_ee);

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
    primp_fused_obj.get_fusion_workspace_density(res_primp_fused.pdf_ee{end}.mean, res_primp_fused.pdf_ee{end}.cov);

%% Compute Yoshimaya manipulability measure
disp('>> Evaluation: manipulability')
metric_primp.manipulability = compute_manipulability_from_ee_pose(res_primp.mean.matrix, mdl_execute, ee_name_execute);
metric_primp_fused.manipulability = compute_manipulability_from_ee_pose(res_primp_fused.mean, mdl_execute, ee_name_execute);

%% Evaluation of ablation study
% Store results as .mat file
res_filename = strcat(result_folder, "result_ablation_primp_wd.mat");
save(res_filename, "res_primp", "res_primp_fused", "metric_primp",...
    "metric_primp_fused");

% Display and store command window
diary_filename = strcat(result_folder, "result_ablation_primp_wd.txt");
diary(diary_filename);

disp('===============================================================')
disp('Ablation results for workspace density adaptation in PRIMP')
disp(['Group: ', group_name])
disp('===============================================================')

disp('>>>> PRIMP with workspace density adaptation <<<<')
disp('---- Manipulability for mean trajectory ----')
disp(num2str( metric_primp_fused.manipulability ))

disp('---------------------------------------------------------------')

disp('>>>> PRIMP (ablated) <<<<')
disp('---- Manipulability for mean trajectory ----')
disp(num2str( metric_primp.manipulability ))

diary off
end