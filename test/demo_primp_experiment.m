% Demo script for PRIMP for real-world experiments
%
%  Data input from "/data/" folder in ".json" format
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of time steps
param.n_step = 50;

% Number of samples from conditional probability
param.n_sample = 10;

% Type of demonstration
demo_type = "pouring/default";

% Group name: 'SE', 'PCG'
param.group_name = 'PCG';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real';

% Data and result folder
param.data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");

% Load demonstrations
filenames = dir(strcat(param.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, param);

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, param.group_name);
n_demo = length(g_demo);

% Load random configurations for conditioning
trials = load_random_trials(result_folder);
n_trial = length(trials.t_via{1});

%% PRIMP main routine
idx = ceil(n_trial * rand);

% Initiate class
tic;

primp_obj = PRIMP(g_mean.matrix, cov_t, param);

% Condition on via-point poses
for i = 1:length(trials.t_via)
    [mu_cond, sigma_cond] = primp_obj.get_condition_pdf(...
        trials.t_via{i}(idx), trials.g_via{i}(:,:,idx),...
        trials.cov_via{i}(:,:,idx));
end
g_samples = primp_obj.get_samples();

toc;

%% PLOT: Condition on via-point poses
frame_scale = 0.1;

figure; hold on; axis off; axis equal;

% Demos
for idx = 1:n_demo
    plot3(g_demo{idx}.pose(1,:), g_demo{idx}.pose(2,:), g_demo{idx}.pose(3,:), 'c')
end

% Prior mean poses
pose_mean = nan(n_step, 7);
for j = 1:n_step
    pose_mean(j,:) = homo2pose_axang(g_mean(:,:,j));
end

plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 3)

% Samples after condition on via pose
for idx = 1:length(trials.g_via)
    trplot(trials.g_via{idx}, 'rviz', 'notext', 'length', frame_scale, 'width', 1)
end

pose_cond_mean = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean(j,:) = homo2pose_axang(mu_cond(:,:,j));
end

plot3(pose_cond_mean(:,1), pose_cond_mean(:,2),...
    pose_cond_mean(:,3), 'm-', 'LineWidth', 3)

%     for j = 1:2:n_step
%         trplot(mu_cond(:,:,j), 'rgb', 'notext', 'length', 0.05)
%     end

pose_samples = generate_pose_struct(g_samples, param.group_name);
for idx = 1:n_sample
    plot3(pose_samples{idx}(:,1), pose_samples{idx}(:,2),...
        pose_samples{idx}(:,3), 'm--', 'LineWidth', 1)
end
