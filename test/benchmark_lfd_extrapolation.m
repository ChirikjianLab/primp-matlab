% Benchmark script for extrapolation between PRIMP and Orientation-KMP
%  Demonstration data input from "/data" folder in ".json" format. Plot 
% results from PRIMP and Orientation-KMP when via poses are out of
% distribution.
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()
addpath ../../../Toolbox/learn_from_demo/pbdlib-matlab/demos/m_fcts/
addpath ../../../Toolbox/learn_from_demo/robInfLib-matlab/fcts/

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of steps for interpolated trajectories
n_step = 50;

% Number of samples from conditional probability
n_sample = 10;

% Number of states in the GMM
n_state = 8;

% KMP parameters
kmp_param.lamda = 1;  % control mean prediction
kmp_param.lamdac = 60; % control variance prediction
kmp_param.kh = 100;

% Name of the dataset
% dataset_name = 'panda_arm';
dataset_name = 'lasa_handwriting/pose_data';

% Type of demonstration
% demo_type = "simulation/circle";
% demo_type = "real/pouring/default";
demo_type = "Snake";
% demo_type = "real/scooping/default";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 200;
VIA_POSE_SCALE.covariance = 1e-4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/", dataset_name, "/", demo_type, "/");
mkdir(result_folder);

%% Load demo data and compute mean/covariance
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate random via/goal poses
trials = generate_random_trials(g_demo{1}, 1, VIA_POSE_SCALE,...
    result_folder);

disp("Generated random configurations!")

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);
n_demo = length(g_demo);

%% Condition on desired goal pose with uncertainty
param.n_sample = n_sample;
param.group_name = group_name;

% Desired via poses and covariance
g_goal = trials.g_via{1};
cov_goal = trials.cov_via{1};
g_via = trials.g_via{2};
cov_via = trials.cov_via{2};
t_via = trials.t_via{2};

% Main routine
% ----- PRIMP
disp('PRIMP')
primp_obj = PRIMP(g_mean.matrix, cov_t, param);

tic;
primp_obj.get_condition_pdf(1.0, g_goal, cov_goal);
[mu_cond_primp, sigma_cond_primp] = primp_obj.get_condition_pdf(t_via,...
    g_via, cov_via);
g_samples_primp = primp_obj.get_samples();
toc;

% Convert to pose
pose_samples_primp = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples_primp{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples_primp{i}(j,:) = homo2pose_quat(g_samples_primp{i}(:,:,j));
    end
end

% Mean and covariance after conditioning
pose_cond_mean_primp = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean_primp(j,:) = homo2pose_axang(mu_cond_primp(:,:,j));
end

sigma_cond_step = zeros(6, 6, n_step);
sigma_cond_step(:,:,1) = cov_t(:,:,1);
for i = 2:n_step-1
    idx_block = 6*(i-2)+1:6*(i-1);
    sigma_cond_step(:,:,i) = sigma_cond_primp(idx_block, idx_block);
end
sigma_cond_step(:,:,end) = cov_goal;

var_cond_step = zeros(n_step, 6);
for i = 1:n_step
    var_cond_step(i,:) = diag(sqrt(sigma_cond_step(:,:,i)));
end

% ----- Orientation-KMP
disp("Orientation-KMP")
param.n_step = n_step;
param.kmp_param = kmp_param;
model.nbStates = n_state;

kmp = KernalizedMovementPrimitives(g_demo, model, param);

tic;

% Condition on goal pose
kmp.compute_kmp_via_point(g_goal, cov_goal, 1.0);
traj_kmp_goal = kmp.get_kmp_trajectory();
g_sample_kmp_goal = kmp.get_samples(traj_kmp_goal, n_sample);

traj_gmr_goal = kmp.get_gmr_trajectory();

% Condition on via pose
kmp.compute_kmp_via_point(g_via, cov_via, t_via);
traj_kmp_via = kmp.get_kmp_trajectory();
g_sample_kmp_via = kmp.get_samples(traj_kmp_via, n_sample);

traj_gmr_via = kmp.get_gmr_trajectory();

toc;

% Extract distributions for GMR and KMP models
[kmp_mean_goal, kmp_cov_goal, kmp_var_goal] = kmp.get_prob_model(traj_kmp_goal);
[kmp_mean_via, kmp_cov_via, kmp_var_via] = kmp.get_prob_model(traj_kmp_via);
[gmr_mean_goal, gmr_cov_goal, gmr_var_goal] = kmp.get_prob_model(traj_gmr_goal);
[gmr_mean_via, gmr_cov_via, gmr_var_via] = kmp.get_prob_model(traj_gmr_via);

% Convert samples to pose
sample_kmp_struct = generate_pose_struct(g_sample_kmp_via,...
            group_name);

%% PLOT: PRIMP
frame_scale = 10;

figure; hold on; axis off; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:), 'c')
end

% Prior mean poses
pose_mean = g_mean.pose;
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 3)

for j = 1:n_step
    trplot(g_mean.matrix(:,:,j), 'rgb', 'notext', 'length', 0.01)
end

% Samples after condition on via pose
plot3(g_goal(1,4), g_goal(2,4), g_goal(3,4), 'k*', LineWidth=3)
plot3(g_via(1,4), g_via(2,4), g_via(3,4), 'k*', LineWidth=3)

plot3(pose_cond_mean_primp(:,1), pose_cond_mean_primp(:,2),...
    pose_cond_mean_primp(:,3), 'm-', 'LineWidth', 3)

%% PLOT: Orientation-KMP
model = kmp.get_gmm_model();

%%%%%%%%%%
figure; hold on; axis equal; axis off;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:), 'c')
end

% Via poses
plot3(g_goal(1,4), g_goal(2,4), g_goal(3,4), 'k*', LineWidth=3)
plot3(g_via(1,4), g_via(2,4), g_via(3,4), 'k*', LineWidth=3)

% GMM
plotGMM3D(model.Mu(5:7,:), model.Sigma(5:7,5:7,:), [0 0 0.8], .2);

% GMR
plot3(gmr_mean_via(4,:), gmr_mean_via(5,:), gmr_mean_via(6,:), 'b-', 'LineWidth', 3);

% KMP
plot3(kmp_mean_via(4,:), kmp_mean_via(5,:), kmp_mean_via(6,:), 'm-', 'LineWidth', 3)