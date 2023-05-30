% Demo script for PRIMP in scooping task
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
% Number of steps for interpolated trajectories
n_step = 50;

% Number of samples from conditional probability
n_sample = 10;

% Name of the dataset
dataset_name = 'panda_arm';

% Type of demonstration
demo_type = "real/scooping/default";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 0.5;
VIA_POSE_SCALE.covariance = 1e-6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/", dataset_name, "/", demo_type, "/");
mkdir(result_folder);

%% Load demo data, generate key poses and compute mean/covariance
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate start, key and goal poses
% Start pose
trials.t_via{1} = 0.0;
[trials.g_via{1}, trials.cov_via{1}, trials.step_via{1}] =...
    generate_random_pose(g_demo{1},...
    trials.t_via{1}, VIA_POSE_SCALE);

% Via pose
trials.t_via{2} = 0.65;
[trials.g_via{2}, trials.cov_via{2}, trials.step_via{2}] =...
    generate_random_pose(g_demo{1},...
    trials.t_via{2}, VIA_POSE_SCALE);

% Goal pose
trials.t_via{3} = 1.0;
[trials.g_via{3}, trials.cov_via{3}, trials.step_via{3}] =...
    generate_random_pose(g_demo{1},...
    trials.t_via{3}, VIA_POSE_SCALE);

disp("Generated key configurations!")

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);
n_demo = length(g_demo);

%% Condition on desired goal pose with uncertainty
param.n_sample = n_sample;
param.group_name = group_name;

% Main routine
primp_obj = PRIMP(g_mean.matrix, cov_t, param);

tic;
for i = 1:length(trials.t_via)
    [mu_cond, sigma_cond] = primp_obj.get_condition_pdf(trials.t_via{i},...
    trials.g_via{i}, trials.cov_via{i});
end
g_samples = primp_obj.get_samples();

toc;

% Convert to pose
pose_samples = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples{i}(j,:) = homo2pose_quat(g_samples{i}(:,:,j));
    end
end

% Mean and covariance after conditioning
pose_cond_mean = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean(j,:) = homo2pose_axang(mu_cond(:,:,j));
end

sigma_cond_step = zeros(6, 6, n_step);
sigma_cond_step(:,:,1) = cov_t(:,:,1);
for i = 2:n_step-1
    idx_block = 6*(i-2)+1:6*(i-1);
    sigma_cond_step(:,:,i) = sigma_cond(idx_block, idx_block);
end
sigma_cond_step(:,:,end) = trials.cov_via{3};

var_cond_step = zeros(n_step, 6);
for i = 1:n_step
    var_cond_step(i,:) = diag(sqrt(sigma_cond_step(:,:,i)));
end

%% PLOT: Demonstrations
figure; hold on; axis off; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:))
end

% Prior mean poses
pose_mean = g_mean.pose;
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 1.5)
plot3(pose_mean(1,1), pose_mean(2,1), pose_mean(3,1), 'go')
plot3(pose_mean(1,end), pose_mean(2,end), pose_mean(3,end), 'r*')

for j = 1:n_step
    trplot(g_mean.matrix(:,:,j), 'rgb', 'notext', 'length', 0.01)
end

%% PLOT: Condition on key poses
frame_scale = 0.03;

figure; hold on; axis off; axis equal;
% Prior mean poses
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 1.5)
plot3(pose_mean(1,1), pose_mean(2,1), pose_mean(3,1), 'go')
plot3(pose_mean(1,end), pose_mean(2,end), pose_mean(3,end), 'r*')

for j = 1:n_step
    trplot(g_mean.matrix(:,:,j), 'rgb', 'notext', 'length', 0.01)
end

% Samples after condition on via poses
for i = 1:length(trials.t_via)
    trplot(trials.g_via{i},...
        'rviz', 'notext', 'length', frame_scale, 'width', 1)
end

plot3(pose_cond_mean(:,1), pose_cond_mean(:,2),...
    pose_cond_mean(:,3), 'm-', 'LineWidth', 3)

for j = 1:n_step
    trplot(mu_cond(:,:,j), 'rgb', 'notext', 'length', 0.01)
end

% for i = 1:n_sample
%     plot3(pose_samples{i}(:,1), pose_samples{i}(:,2),...
%         pose_samples{i}(:,3), 'm--', 'LineWidth', 1)
% end