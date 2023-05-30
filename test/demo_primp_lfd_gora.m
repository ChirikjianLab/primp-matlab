% Demo script for PRIMP using GORA as pre-process
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
% dataset_name = 'lasa_handwriting/pose_data';

% Type of demonstration
demo_type = "simulation/circle";
% demo_type = "real/pouring/default";
% demo_type = "Snake";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 1;
VIA_POSE_SCALE.covariance = 1e-4;

% Indicator of whether to generate random via/goal poses
is_generate_random = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");

%% Load demo data and compute mean/covariance
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));

% Parse demo trajectories
% with GORA
argin.align_method = 'gora';
g_demo = parse_demo_trajectory(filenames, argin);

% without GORA
argin.align_method = 'interp';
g_demo_no_gora = parse_demo_trajectory(filenames, argin);

n_demo = length(g_demo);

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);
[g_mean_no_gora, cov_t_no_gora] = get_pdf_from_demo(g_demo_no_gora,...
    group_name);

%% Condition on desired goal pose with uncertainty
param.n_sample = n_sample;
param.group_name = group_name;

% Generate random via/goal poses
disp("Generate random configurations")
trials = generate_random_trials(g_demo{1}, 1, VIA_POSE_SCALE);
g_goal = trials.g_via{1};
cov_goal = trials.cov_via{1};

% Main routine
primp_obj = PRIMP(g_mean.matrix, cov_t, param);
primp_no_gora_obj = PRIMP(g_mean_no_gora.matrix, cov_t_no_gora, param);

tic;
[mu_cond, sigma_cond] = primp_obj.get_condition_pdf(1.0,...
    g_goal, cov_goal);
g_samples = primp_obj.get_samples();
toc;

tic;
[mu_cond_no_gora, sigma_cond_no_gora] =...
    primp_no_gora_obj.get_condition_pdf(1.0, g_goal, cov_goal);
g_samples_no_gora = primp_no_gora_obj.get_samples();
toc;

% Convert to pose
pose_samples = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples{i}(j,:) = homo2pose_quat(g_samples{i}(:,:,j));
    end
end

pose_samples_no_gora = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples_no_gora{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples_no_gora{i}(j,:) = homo2pose_quat(...
            g_samples_no_gora{i}(:,:,j));
    end
end

% Mean and covariance after conditioning
pose_cond_mean = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean(j,:) = homo2pose_axang(mu_cond(:,:,j));
end

pose_cond_mean_no_gora = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean_no_gora(j,:) = homo2pose_axang(mu_cond_no_gora(:,:,j));
end

%% PLOT: Demonstrations with/without GORA
figure; hold on; axis off; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:))
end

% Prior mean poses
pose_mean = g_mean.pose;
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 1.5)

% Demos without GORA
for i = 1:n_demo
    plot3(g_demo_no_gora{i}.pose(1,:), g_demo_no_gora{i}.pose(2,:),...
        g_demo_no_gora{i}.pose(3,:))
end

% Prior mean poses without GORA
pose_mean_no_gora = g_mean_no_gora.pose;
plot3(pose_mean_no_gora(1,:), pose_mean_no_gora(2,:),...
    pose_mean_no_gora(3,:), 'r--', 'LineWidth', 1.5)

%% PLOT: Condition on goal pose
frame_scale = 0.1;

figure; hold on; axis off; axis equal;
% Prior mean poses
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 3)
plot3(pose_mean_no_gora(1,:), pose_mean_no_gora(2,:),...
    pose_mean_no_gora(3,:), 'r--', 'LineWidth', 3)

% Samples after conditioning
trplot(g_goal, 'rviz', 'notext', 'length', frame_scale, 'width', 1)

plot3(pose_cond_mean(:,1), pose_cond_mean(:,2),...
    pose_cond_mean(:,3), 'c-', 'LineWidth', 3)
plot3(pose_cond_mean_no_gora(:,1), pose_cond_mean_no_gora(:,2),...
    pose_cond_mean_no_gora(:,3), 'm--', 'LineWidth', 3)

%% PLOT: Trajectory profile for demonstrations and PRIMP encoding
figure;

t_steps = 0:1/(n_step-1):1;

% For translation part
subplot(2,1,1); grid on; hold on;

plot(t_steps, g_mean.exponential(4,:),...
    t_steps, g_mean.exponential(5,:),...
    t_steps, g_mean.exponential(6,:), 'LineWidth', 1)

plot(t_steps, g_mean_no_gora.exponential(4,:), '--',...
    t_steps, g_mean_no_gora.exponential(5,:), '--',...
    t_steps, g_mean_no_gora.exponential(6,:), '--',...
    'LineWidth', 1)

title('Translation part')
xlabel('Time')

% For rotation part, in exponential coordinates
subplot(2,1,2); grid on; hold on;

plot(t_steps, g_mean.exponential(1,:),...
    t_steps, g_mean.exponential(2,:),...
    t_steps, g_mean.exponential(3,:), 'LineWidth', 1)

plot(t_steps, g_mean_no_gora.exponential(1,:), '--',...
    t_steps, g_mean_no_gora.exponential(2,:), '--',...
    t_steps, g_mean_no_gora.exponential(3,:), '--',...
    'LineWidth', 1)

title('Rotation part, in Lie algebra so(3)')
xlabel('Time')

%% PLOT: Trajectory profile
figure;

% Convert to Lie algebra
t_steps = 0:1/(n_step-1):1;
exp_goal = get_exp_coord(g_goal, argin.group_name);

primp_mean_via = zeros(n_step, 6);
for i = 1:n_step
    primp_mean_via(i,:) = get_exp_coord(mu_cond(:,:,i), argin.group_name);
end

primp_mean_via_no_gora = zeros(n_step, 6);
for i = 1:n_step
    primp_mean_via_no_gora(i,:) = get_exp_coord(mu_cond_no_gora(:,:,i),...
        argin.group_name);
end

% For translation part
subplot(2,1,1); grid on; hold on;

plot(1, exp_goal(4), 'o', 1, exp_goal(5), 'o', 1, exp_goal(6), 'o')
plot(t_steps, primp_mean_via(:,4), 'b-', t_steps, primp_mean_via(:,5),...
    'b-', t_steps, primp_mean_via(:,6), 'b-', 'LineWidth', 3)
plot(t_steps, primp_mean_via_no_gora(:,4), 'r--', t_steps,...
    primp_mean_via_no_gora(:,5), 'r--', t_steps,...
    primp_mean_via_no_gora(:,6), 'r--', 'LineWidth', 3)

title('Translation part')
xlabel('Time')

% For rotation part, in exponential coordinates
subplot(2,1,2); grid on; hold on;

plot(1, exp_goal(1), 'o', 1, exp_goal(2), 'o', 1, exp_goal(3), 'o')
plot(t_steps, primp_mean_via(:,1), 'b-', t_steps, primp_mean_via(:,2),...
    'b-', t_steps, primp_mean_via(:,3), 'b-', 'LineWidth', 3)
plot(t_steps, primp_mean_via_no_gora(:,1), 'r--', t_steps,...
    primp_mean_via_no_gora(:,2), 'r--', t_steps,...
    primp_mean_via_no_gora(:,3), 'r--', 'LineWidth', 3)

title('Rotation part, in Lie algebra so(3)')
xlabel('Time')