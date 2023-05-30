% Demonstration script for Orientation-KMP
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths();
addpath ../../../Toolbox/learn_from_demo/pbdlib-matlab/demos/m_fcts/
addpath ../../../Toolbox/learn_from_demo/robInfLib-matlab/fcts/

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of time steps
n_step = 50;

% Number of sampled trajectories from distribution
n_sample = 5;

% Number of states in the GMM
n_state = 8;

% KMP parameters
kmp_param.lamda = 1;  % control mean prediction
kmp_param.lamdac = 60; % control variance prediction
kmp_param.kh = 6;

% Type of demonstration
demo_type = "simulation/circle";

% Name of the dataset
dataset_name = 'panda_arm';

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 1;
VIA_POSE_SCALE.covariance = 1e-5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");

%% Load demo data
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate random via/goal poses
trials = generate_random_trials(g_demo{1}, 1, VIA_POSE_SCALE);
n_demo = length(g_demo);
disp("Generated random configurations!")

% Load random via/goal poses
g_goal = trials.g_via{1};
cov_goal = trials.cov_via{1};

t_via = trials.t_via{2};
g_via = trials.g_via{2};
cov_via = trials.cov_via{2};

%% Learning KMP model from demos
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

%% Plots
model = kmp.get_gmm_model();

%%%%%%%%%%
figure; hold on; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:))
end

% Via poses
plot3(g_via(1,4), g_via(2,4), g_via(3,4), 'o', 'LineWidth', 1.5)
plot3(g_goal(1,4), g_goal(2,4), g_goal(3,4), '*', 'LineWidth', 1.5)

% GMM
plotGMM3D(model.Mu(5:7,:), model.Sigma(5:7,5:7,:), [.8 0 0], .5);

% GMR
plot3(gmr_mean_via(4,:), gmr_mean_via(5,:), gmr_mean_via(6,:), 'k--', 'LineWidth', 1.5);

% KMP
plot3(kmp_mean_via(4,:), kmp_mean_via(5,:), kmp_mean_via(6,:), 'b-', 'LineWidth', 1.5)

%%%%%%%%%%
figure; hold on; axis equal;

% Via poses
plot3(g_via(1,4), g_via(2,4), g_via(3,4), 'o', 'LineWidth', 1.5)
plot3(g_goal(1,4), g_goal(2,4), g_goal(3,4), '*', 'LineWidth', 1.5)

% GMM
plotGMM3D(model.Mu(5:7,:), model.Sigma(5:7,5:7,:), [.8 0 0], .5);

% GMR
plot3(gmr_mean_via(4,:), gmr_mean_via(5,:), gmr_mean_via(6,:), 'k--', 'LineWidth', 1.5);

% KMP
plot3(kmp_mean_via(4,:), kmp_mean_via(5,:), kmp_mean_via(6,:), 'b-', 'LineWidth', 1.5)

plot3(kmp_mean_via(4,:) + kmp_var_via(4,:), kmp_mean_via(5, :) + kmp_var_via(5,:),...
    kmp_mean_via(6,:) + kmp_var_via(6,:), 'm--', 'LineWidth', 1.5)
plot3(kmp_mean_via(4,:) - kmp_var_via(4,:), kmp_mean_via(5,:) - kmp_var_via(5,:),...
    kmp_mean_via(6,:) - kmp_var_via(6,:), 'm--', 'LineWidth', 1.5)

%%%%%%%%%%
% Trajectory profile
figure;

t_steps = 0:1/(n_step-1):1;
exp_via = get_exp_coord(g_via, group_name);
exp_goal = get_exp_coord(g_goal, group_name);

% For translation part
subplot(2,1,1); hold on;

plot(t_via, exp_via(4), 'o', t_via, exp_via(5), 'o',...
    t_via, exp_via(6), 'o')
plot(1, exp_goal(4), '*', 1, exp_goal(5), '*', 1, exp_goal(6), '*')
plot(t_steps, kmp_mean_via(4,:), t_steps, kmp_mean_via(5,:),...
    t_steps, kmp_mean_via(6,:), 'LineWidth', 1.5)

plot(t_steps, kmp_mean_via(4,:) + kmp_var_via(4,:), 'm--',...
    t_steps, kmp_mean_via(5,:) + kmp_var_via(5,:), 'm--',...
    t_steps, kmp_mean_via(6,:) + kmp_var_via(6,:), 'm--',...
    'LineWidth', 1.5)
plot(t_steps, kmp_mean_via(4,:) - kmp_var_via(4,:), 'm--',...
    t_steps, kmp_mean_via(5,:) - kmp_var_via(5,:), 'm--',...
    t_steps, kmp_mean_via(6,:) - kmp_var_via(6,:), 'm--',...
    'LineWidth', 1.5)

title('Translation part')
xlabel('Time')

% For rotation part, in exponential coordinates
subplot(2,1,2); hold on;

plot(t_via, exp_via(1), 'o', t_via, exp_via(2), 'o',...
    t_via, exp_via(3), 'o')
plot(1, exp_goal(1), '*', 1, exp_goal(2), '*', 1, exp_goal(3), '*')
plot(t_steps, kmp_mean_via(1,:), t_steps, kmp_mean_via(2,:),...
    t_steps, kmp_mean_via(3,:), 'LineWidth', 1.5)

plot(t_steps, kmp_mean_via(1,:) + kmp_var_via(1,:), 'm--',...
    t_steps, kmp_mean_via(2,:) + kmp_var_via(2,:), 'm--',...
    t_steps, kmp_mean_via(3,:) + kmp_var_via(3,:), 'm--',...
    'LineWidth', 1.5)
plot(t_steps, kmp_mean_via(1,:) - kmp_var_via(1,:), 'm--',...
    t_steps, kmp_mean_via(2,:) - kmp_var_via(2,:), 'm--',...
    t_steps, kmp_mean_via(3,:) - kmp_var_via(3,:), 'm--',...
    'LineWidth', 1.5)

title('Rotation part, in so(3)')
xlabel('Time')