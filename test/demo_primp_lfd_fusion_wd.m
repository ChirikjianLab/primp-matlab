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

% Number of samples from conditional probability
n_sample = 5;

% Number of sampled angles for each joint
n_state = 1000;

% Type of demonstration
demo_type = "simulation/circle";
% demo_type = "real/pouring/default";

% Name of the dataset
dataset_name = 'panda_arm';

% Name of the robot for execution
% robot_execute = 'panda_arm';
robot_execute = 'kinovaGen3';
% robot_execute = 'robotisOpenManipulator';

% Group name: 'SE', 'PCG'
group_name = 'SE';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 1;
VIA_POSE_SCALE.covariance = 1e-5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load demo data and compute mean/covariance
data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");

argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate random via/goal poses
trials = generate_random_trials(g_demo{1}, 1, VIA_POSE_SCALE);

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);

%% Condition on desired goal pose with uncertainty
disp('Condition on via pose')

% Desired goal pose and covariance
g_goal = trials.g_via{1};
cov_goal = trials.cov_via{1};

% Condition on via pose
param.n_sample = n_sample;
param.group_name = group_name;

primp_obj = PRIMP(g_mean.matrix, cov_t, param);
g_samples_origin = primp_obj.get_samples();

[mu_cond, sigma_cond] = primp_obj.get_condition_pdf(1.0, g_goal, cov_goal);
g_samples = primp_obj.get_samples();

%% Fuse with workspace density of robot arm
disp('Fusion with workspace density')

% Poses of end effector
[pose_ee, robot_model, ee_name] = generate_robot_ee_pose(n_state, robot_execute);

% Mean and covariance of the poses of each link
pdf_ee = get_pdf_from_pose_SE(pose_ee);

primp_wd_obj = PRIMP(g_mean.matrix, cov_t, param);

tic;
% Fuse with workspace density
[mu_fused, sigma_fused] = primp_wd_obj.get_fusion_workspace_density(...
    pdf_ee{end}.mean, pdf_ee{end}.cov);

% Condition on via pose
mu_cond_fused = primp_wd_obj.get_condition_pdf(1.0, g_goal, cov_goal);
g_samples_fused = primp_wd_obj.get_samples();
toc;

%% Convert to pose
pose_samples_origin = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples_origin{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples_origin{i}(j,:) =...
            homo2pose_quat(g_samples_origin{i}(:,:,j));
    end
end

pose_samples_fused = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples_fused{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples_fused{i}(j,:) =...
            homo2pose_quat(g_samples_fused{i}(:,:,j));
    end
end

pose_fused_mean = nan(n_step, 7);
for j = 1:n_step
    pose_fused_mean(j,:) = homo2pose_axang(mu_fused(:,:,j));
end

pose_samples = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples{i}(j,:) = homo2pose_quat(g_samples{i}(:,:,j));
    end
end

pose_cond_mean = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean(j,:) = homo2pose_axang(mu_cond(:,:,j));
end

%% Manipulability
disp('Manipulability (averaged)')
m_before = compute_manipulability_from_ee_pose(g_mean.matrix, robot_model, ee_name);
m_after = compute_manipulability_from_ee_pose(mu_fused, robot_model, ee_name);

disp([' -- before: ', num2str(mean(m_before)), '; after: ', num2str(mean(m_after))])

%% Plots
figure;

ik = inverseKinematics('RigidBodyTree', robot_model);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = robot_model.homeConfiguration;

for i = 35
    [configSol, solInfo] = ik(ee_name, mu_fused(:,:,i),...
        weights, initialguess);
    robot_model.show(configSol, 'Frames', 'off');

    hold on; axis equal; axis off;
end


% Samples after conditioning
trplot(g_goal, 'rviz', 'notext', 'length', 0.1, 'width', 1)

for i = 1:n_sample
    % Original probability
    plot3(pose_samples_origin{i}(:,1), pose_samples_origin{i}(:,2),...
        pose_samples_origin{i}(:,3))

    % Conditional probability
    plot3(pose_cond_mean(:,1), pose_cond_mean(:,2),...
        pose_cond_mean(:,3), 'b-', 'LineWidth', 3)

    %     plot3(pose_samples{i}(:,1), pose_samples{i}(:,2),...
    %         pose_samples{i}(:,3), 'm-')

    % Probability fused with workspace density of robot arm
    plot3(pose_fused_mean(:,1), pose_fused_mean(:,2),...
        pose_fused_mean(:,3), 'm-', 'LineWidth', 3)

    %     plot3(pose_samples_fused{i}(:,1), pose_samples_fused{i}(:,2),...
    %         pose_samples_fused{i}(:,3), 'c-')
end

% Sample from robot workspace density
n_wd_sample = 1000;

mean_end_exp = get_exp_coord(pdf_ee{end}.mean);
exp_sample = mvnrnd(mean_end_exp, pdf_ee{end}.cov, n_wd_sample)';

x_sample = nan(7, n_wd_sample);
for i = 1:n_wd_sample
    g = get_exp_mapping(exp_sample(:,i));
    x_sample(:,i) = [g(1:3,4); rotm2quat(g(1:3,1:3))'];
end

plot3(x_sample(1,:), x_sample(2,:), x_sample(3,:), 'r.')