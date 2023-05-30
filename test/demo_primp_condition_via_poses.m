% Demonstration script for PRIMP that is conditioned on SE(3) via poses 
% with uncertainty.
%
%  Author
%    Sipu Ruan, 2022

close all; clear; clc;
add_paths()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of time steps
n_step = 30;

% Number of sampled trajectories from distribution
n_sample = 10;

% Group name: 'SE', 'PCG'
group_name = 'SE';

% Scaling of initial covariance
COV_INIT_SCALE = 1e-4;

% Via pose deviation from initial, in exponential coordinates
VIA_POSE_DEVIATION_EXP_COORD = [pi*1e-2*rand(3,1); 0.1*rand(3,1)];

% Scaling of via pose covariance
COV_VIA_POSE_SCALE = 1e-5;

% Time step for the via pose
T_VIA = 0.43;

% New frame of viewing point
H_VIEW = [axang2rotm(pi*rand(1,4)), rand(3,1); 0, 0, 0, 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cov_dim = 6;

%% Simulate robot workspace trajectory distribution
argin.n_step = n_step;
argin.group_name = group_name;
argin.cov_dim = cov_dim;
argin.cov_init_scale = COV_INIT_SCALE;
argin.robot = loadrobot('frankaEmikaPanda', "Gravity", [0 0 -9.81]);

[g_mean, cov_t] = generate_trajectory(argin);

%% Workspace density using conditional PDF in SE(3)
% Desired via pose and covariance
step_via = floor(T_VIA*n_step);
if step_via <= 0
    step_via = 1;
elseif step_via >= n_step
    step_via = n_step;
end

g_via = g_mean.matrix(:,:,step_via) * expm_SE(VIA_POSE_DEVIATION_EXP_COORD);
cov_via = COV_VIA_POSE_SCALE * diag([4 4 4 1 1 1]);
g_goal = g_mean.matrix(:,:,end) * expm_SE(VIA_POSE_DEVIATION_EXP_COORD);
cov_goal = COV_VIA_POSE_SCALE * diag([4 4 4 1 1 1]);

% Initiate class
param.n_sample = n_sample;
param.group_name = group_name;

primp_obj = PRIMP(g_mean.matrix, cov_t, param);

% Using Density class methods
tic;

g_samples_matrix = primp_obj.get_samples();
g_samples_0 = generate_pose_struct(g_samples_matrix, param.group_name);

% Condition on goal pose
[mu_cond_1, sigma_cond_1] = primp_obj.get_condition_pdf(1.0,...
    g_goal, cov_goal);
g_samples_matrix = primp_obj.get_samples();
g_samples_1 = generate_pose_struct(g_samples_matrix, param.group_name);

% Condition on a via pose
[mu_cond_2, sigma_cond_2] = primp_obj.get_condition_pdf(T_VIA,...
    g_via, cov_via);
g_samples_matrix = primp_obj.get_samples();
g_samples_2 = generate_pose_struct(g_samples_matrix, param.group_name);

toc;

%% PLOT: Original trajectory samples
figure; hold on; axis equal; axis off;

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:), g_mean.pose(3,:),...
    'b-', 'LineWidth', 3)

plot3(g_mean.pose(1,1), g_mean.pose(2,1), g_mean.pose(3,1), 'go')
plot3(g_mean.pose(1,end), g_mean.pose(2,end), g_mean.pose(3,end), 'r*')

% Samples from joint distribution
for i = 1:n_sample
    plot3(g_samples_0{i}.pose(1,:),...
        g_samples_0{i}.pose(2,:),...
        g_samples_0{i}.pose(3,:), 'm--')
end

%% PLOT: Condition on goal pose
figure; hold on; axis equal; axis off;

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:),...
    g_mean.pose(3,:), 'b-', 'LineWidth', 3)

plot3(g_mean.pose(1,1), g_mean.pose(2,1),...
    g_mean.pose(3,1), 'go')
plot3(g_mean.pose(1,end), g_mean.pose(2,end),...
    g_mean.pose(3,end), 'r*')

% Samples after conditioning
trplot(g_goal, 'rviz', 'notext', 'length', 0.1, 'thick', 0.1)

% Mean after conditioning
mu_cond_cartesian = nan(7,n_step);
for j = 1:n_step
    mu_cond_cartesian(:,j) = [mu_cond_1(1:3,4,j);
        rotm2quat(mu_cond_1(1:3,1:3,j))'];
end
plot3(mu_cond_cartesian(1,:), mu_cond_cartesian(2,:),...
    mu_cond_cartesian(3,:), 'm-', 'LineWidth', 3)

for i = 1:n_sample
    plot3(g_samples_1{i}.pose(1,:),...
        g_samples_1{i}.pose(2,:),...
        g_samples_1{i}.pose(3,:), 'm--')
end

%% PLOT: Condition on goal and a via pose
figure; hold on; axis equal; axis off;

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:),...
    g_mean.pose(3,:), 'b-', 'LineWidth', 3)

plot3(g_mean.pose(1,1), g_mean.pose(2,1),...
    g_mean.pose(3,1), 'go')
plot3(g_mean.pose(1,end), g_mean.pose(2,end),...
    g_mean.pose(3,end), 'r*')

% Samples after conditioning
trplot(g_goal, 'rviz', 'notext', 'length', 0.1, 'thick', 0.1)
trplot(g_via, 'rviz', 'notext', 'length', 0.1, 'thick', 0.1)

% Mean after conditioning
mu_cond_cartesian = nan(7,n_step);
for j = 1:n_step
    mu_cond_cartesian(:,j) = [mu_cond_2(1:3,4,j);
        rotm2quat(mu_cond_2(1:3,1:3,j))'];
end
plot3(mu_cond_cartesian(1,:), mu_cond_cartesian(2,:),...
    mu_cond_cartesian(3,:), 'm-', 'LineWidth', 3)

for i = 1:n_sample
    plot3(g_samples_2{i}.pose(1,:),...
        g_samples_2{i}.pose(2,:),...
        g_samples_2{i}.pose(3,:), 'm--')
end