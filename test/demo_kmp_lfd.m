% Demonstration script for Orientation-KMP
%
%  Author
%    Sipu Ruan, 2023
%
%  Reference (modified parts of the code)
%   - PbDLib: https://gitlab.idiap.ch/rli/pbdlib-matlab/
%   - Orientation-KMP: https://github.com/yanlongtu/robInfLib-matlab

close all; clear; clc;
add_paths();
addpath ../src/external/pbdlib-matlab/demos/m_fcts/
addpath ../src/external/robInfLib-matlab/fcts/

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
kmp_param.lamda = 0.1;  % control mean prediction
kmp_param.lamdac = 60; % control variance prediction
kmp_param.kh = 10;

% Name of the dataset
dataset_name = "lasa_handwriting/pose_data";
% dataset_name = 'panda_arm';

% Type of demonstration
demo_type = "Snake";
% demo_type = "simulation/circle";
% demo_type = "real/pouring/default";

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = [1e-3 * ones(3,1); 1e-4 * ones(3,1)];
VIA_POSE_SCALE.covariance = 1e-5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");

% Group is fixed as PCG, since translation and rotation are learned
% separately
group_name = 'PCG';

%% Load data
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;
argin.align_method = "interp";

% Load and parse demo data
filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate random via/goal poses
t_via = [0, 1];
trials = generate_random_trials(g_demo{1}, t_via, VIA_POSE_SCALE);
n_demo = length(g_demo);
disp("Generated random configurations!")

% Load random via/goal poses
g_via_1 = trials.g_via{1};
cov_via_1 = trials.cov_via{1};
g_via_2 = trials.g_via{2};
cov_via_2 = trials.cov_via{2};

%% Learning KMP model from demos
param.n_step = n_step;
param.kmp_param = kmp_param;
model.nbStates = n_state;

tic;

% Construct class and learn GMM/GMR model
kmp_obj = kmp(g_demo, model, param);

% Condition on goal pose
kmp_obj.compute_kmp_via_point(g_via_1, cov_via_1, t_via(1));
traj_kmp_goal = kmp_obj.get_kmp_trajectory();
g_sample_kmp_goal = kmp_obj.get_samples(traj_kmp_goal, n_sample);

traj_gmr_goal = kmp_obj.get_gmr_trajectory();

% Condition on via pose
kmp_obj.compute_kmp_via_point(g_via_2, cov_via_2, t_via(2));
traj_kmp_via = kmp_obj.get_kmp_trajectory();
g_sample_kmp_via = kmp_obj.get_samples(traj_kmp_via, n_sample);

traj_gmr_via = kmp_obj.get_gmr_trajectory();

toc;

% Extract distributions for GMR and KMP models
[kmp_mean_goal, kmp_cov_goal, kmp_var_goal] = kmp_obj.get_prob_model(traj_kmp_goal);
[kmp_mean_via, kmp_cov_via, kmp_var_via] = kmp_obj.get_prob_model(traj_kmp_via);
[gmr_mean_goal, gmr_cov_goal, gmr_var_goal] = kmp_obj.get_prob_model(traj_gmr_goal);
[gmr_mean_via, gmr_cov_via, gmr_var_via] = kmp_obj.get_prob_model(traj_gmr_via);

% Convert samples to pose
sample_kmp_struct = generate_pose_struct(g_sample_kmp_via,...
            group_name);

%% Plots
model = kmp_obj.get_gmm_model();

%%%%%%%%%%
figure; hold on; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:))
end

% Via poses
plot3(g_via_1(1,4), g_via_1(2,4), g_via_1(3,4), '*', 'LineWidth', 1.5)
plot3(g_via_2(1,4), g_via_2(2,4), g_via_2(3,4), 'o', 'LineWidth', 1.5)

% GMM
plotGMM3D(model.Mu(5:7,:), model.Sigma(5:7,5:7,:), [.8 0 0], .5);

% GMR
plot3(gmr_mean_via(4,:), gmr_mean_via(5,:), gmr_mean_via(6,:), 'k--', 'LineWidth', 1.5);

% KMP
plot3(kmp_mean_via(4,:), kmp_mean_via(5,:), kmp_mean_via(6,:), 'b-', 'LineWidth', 1.5)

%%%%%%%%%%
figure; hold on; axis equal;

% Via poses
plot3(g_via_1(1,4), g_via_1(2,4), g_via_1(3,4), '*', 'LineWidth', 1.5)
plot3(g_via_2(1,4), g_via_2(2,4), g_via_2(3,4), 'o', 'LineWidth', 1.5)

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
exp_via_1 = get_exp_coord(g_via_1, group_name);
exp_via_2 = get_exp_coord(g_via_2, group_name);

% For translation part
subplot(2,1,1); hold on;

% Demos
for i = 1:n_demo
    plot(t_steps, g_demo{i}.exponential(4,:),...
        t_steps, g_demo{i}.exponential(5,:),...
        t_steps, g_demo{i}.exponential(6,:), 'k')
end

plot(t_via(1), exp_via_1(4), '*', t_via(1), exp_via_1(5), '*',...
    t_via(1), exp_via_1(6), '*')
plot(t_via(2), exp_via_2(4), 'o', t_via(2), exp_via_2(5), 'o',...
    t_via(2), exp_via_2(6), 'o')
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

% Demos
for i = 1:n_demo
    plot(t_steps, g_demo{i}.exponential(1,:),...
        t_steps, g_demo{i}.exponential(2,:),...
        t_steps, g_demo{i}.exponential(3,:), 'k')
end

plot(t_via(1), exp_via_1(1), '*', t_via(1), exp_via_1(2), '*',...
    t_via(1), exp_via_1(3), '*')
plot(t_via(2), exp_via_2(1), 'o', t_via(2), exp_via_2(2), 'o',...
    t_via(2), exp_via_2(3), 'o')
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

%%%%%%%%%%
% Trajectory profile
figure;

t_steps = 0:1/(n_step-1):1;
exp_via_1 = get_exp_coord(g_via_1, group_name);
exp_via_2 = get_exp_coord(g_via_2, group_name);

% For translation part
subplot(2,1,1); hold on;

plot(t_via(1), exp_via_1(4), '*', t_via(1), exp_via_1(5), '*',...
    t_via(1), exp_via_1(6), '*')
plot(t_via(2), exp_via_2(4), 'o', t_via(2), exp_via_2(5), 'o',...
    t_via(2), exp_via_2(6), 'o')
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

plot(t_via(1), exp_via_1(1), '*', t_via(1), exp_via_1(2), '*',...
    t_via(1), exp_via_1(3), '*')
plot(t_via(2), exp_via_2(1), 'o', t_via(2), exp_via_2(2), 'o',...
    t_via(2), exp_via_2(3), 'o')
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