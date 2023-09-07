% Demonstration script for PRIMP. Illustration for fusion with
% robot-specific workspace density.
%
%  Data input from "/data/" folder in ".json" format
%
%  Author
%    Sipu Ruan, 2022

close all; clear; clc;
add_paths()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of steps for interpolated trajectories
n_step = 50;

% Number of sampled angles for each joint
n_state = 1000;

% Type of demonstration
demo_type = "simulation/letter_S";
% demo_type = "real/pouring/default";

% Name of the dataset
dataset_name = 'panda_arm';

% Name of the robot for execution
% robot_execute = 'panda_arm';
robot_execute = 'kinovaGen3';
% robot_execute = 'ur5';
% robot_execute = 'kukaIiwa7';

% Group name: 'SE', 'PCG'
group_name = 'PCG';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load demo data and compute mean/covariance
data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");

param.n_step = n_step;
param.data_folder = data_folder;
param.group_name = group_name;
param.n_sample = 50;

filenames = dir(strcat(param.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, param);
n_demo = length(g_demo);

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);

%% Fuse with workspace density of robot arm
disp('Fusion with workspace density')

% Poses of distal end of each link for the manipulator
[pose_ee, mdl_execute, ee_name_execute] = generate_robot_ee_pose(n_state, robot_execute);

% Mean and covariance of the poses of each link
pdf_ee = get_pdf_from_pose(pose_ee, group_name);
pdf_ee = pdf_ee{end};

% Fuse with workspace density
primp_wd_obj = PRIMP(g_mean.matrix, cov_t, param);
[mu_fused, sigma_fused] = primp_wd_obj.get_fusion_workspace_density(...
    pdf_ee.mean, pdf_ee.cov);

%% Convert to pose
pose_mean = nan(n_step, 7);
for j = 1:n_step
    pose_mean(j,:) = homo2pose_axang(g_mean.matrix(:,:,j));
end

pose_fused_mean = nan(n_step, 7);
for j = 1:n_step
    pose_fused_mean(j,:) = homo2pose_axang(mu_fused(:,:,j));
end

%% Compute Mahalanobis distance with workspace density
disp('Manipulability measure')
maniplty_primp_fused = compute_manipulability_from_ee_pose(mu_fused, mdl_execute, ee_name_execute);
maniplty_primp = compute_manipulability_from_ee_pose(g_mean.matrix, mdl_execute, ee_name_execute);

disp(['-- with WD: ', num2str(mean(maniplty_primp_fused)), '; without WD: ', num2str(mean(maniplty_primp))]);

%% Compute Mahalanobis distance with workspace density
disp('Mahalanobis distance to workspace density')
dist_primp_fused = compute_mahalanobis_distance(mu_fused, pdf_ee.mean, pdf_ee.cov, group_name);
dist_primp = compute_mahalanobis_distance(g_mean.matrix, pdf_ee.mean, pdf_ee.cov, group_name);

disp(['-- with WD: ', num2str(mean(dist_primp_fused)), '; without WD: ', num2str(mean(dist_primp))]);

%% Plots
figure;

ik = inverseKinematics('RigidBodyTree', mdl_execute);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = mdl_execute.homeConfiguration;

for i = 35
    [configSol, solInfo] = ik(ee_name_execute, mu_fused(:,:,i),...
        weights, initialguess);
    mdl_execute.show(configSol, 'Frames', 'off');

    hold on; axis equal; axis off;
end

% Demo trajectories
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:))
end

% Conditional probability
plot3(pose_mean(:,1), pose_mean(:,2), pose_mean(:,3), 'b-', 'LineWidth', 3)

% Probability fused with workspace density of robot arm
plot3(pose_fused_mean(:,1), pose_fused_mean(:,2), pose_fused_mean(:,3),...
    'm-', 'LineWidth', 3)

% Samples from robot workspace density
x_sample = pose_ee{1}.cartesian;
plot3(x_sample(1,:), x_sample(2,:), x_sample(3,:), 'r.')

% Gaussian for orkspace density
plot_gaussian_pdf(pdf_ee.mean, pdf_ee.cov);