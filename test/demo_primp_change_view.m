% Demonstration script for PRIMP that is conditioned on SE(3) via poses 
% with uncertainty, under the change of viewing frame. An illustration for 
% equivariance property.
%
%  Author
%    Sipu Ruan, 2022

close all; clear; clc;
add_paths()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of time steps
n_step = 50;

% Number of sampled trajectories from distribution
n_sample = 50;

% Group name: 'SE', 'PCG'
group_name = 'SE';

% Scaling of initial covariance
COV_INIT_SCALE = 1e-4;

% Time step of via pose
T_VIA = 0.6;

% Via pose deviation from initial, in exponential coordinates
VIA_POSE_DEVIATION_EXP_COORD = [pi*1e-2*rand(3,1); 0.2*rand(3,1)];

% Scaling of goal pose covariance
COV_VIA_POSE_SCALE = 1e-6;

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
% Initiate class
param.n_sample = n_sample;
param.group_name = group_name;

%%%%% Fixed goal
% Original frame
primp_obj = PRIMP(g_mean.matrix, cov_t, param);
tic;
sigma_joint = primp_obj.get_joint_pdf();
g_samples_matrix = primp_obj.get_samples();
toc;
g_samples_joint = generate_pose_struct(g_samples_matrix, param.group_name);

% Change of view
primp_obj_view = PRIMP(g_mean.matrix, cov_t, param);
primp_obj_view.set_new_view_frame(H_VIEW);
tic;
sigma_joint_view = primp_obj_view.get_joint_pdf();
g_samples_matrix = primp_obj_view.get_samples();
toc;

% Change viewing frame back for validation
for i = 1:n_sample
    for j = 1:n_step
        g_samples_matrix{i}(:,:,j) = H_VIEW *...
            g_samples_matrix{i}(:,:,j) / H_VIEW;
    end
end
g_samples_joint_view = generate_pose_struct(g_samples_matrix, param.group_name);

%%%%%% Desired via pose and covariance
t_idx = floor(T_VIA * n_step);
if t_idx <= 0
    t_idx = 1;
elseif t_idx >= n_step
    t_idx = n_step;
end

g_via = g_mean.matrix(:,:,t_idx) * expm_SE(VIA_POSE_DEVIATION_EXP_COORD);
cov_via = COV_VIA_POSE_SCALE * diag([4 4 4 1 1 1]);

% Identity frame
tic;
[mu_cond, sigma_cond] = primp_obj.get_condition_pdf(T_VIA, g_via, cov_via);
g_samples_matrix = primp_obj.get_samples();
toc;
g_samples = generate_pose_struct(g_samples_matrix, param.group_name);

% Change of view
tic;
[mu_cond_view, sigma_cond_view] = primp_obj_view.get_condition_pdf(T_VIA, g_via, cov_via);
g_samples_matrix = primp_obj_view.get_samples();
toc;

mu_cond_view_cartesian = nan(7,n_step);
mu_cond_view_back = nan(4,4,n_step);
for j = 1:n_step
    mu_cond_view_back(:,:,j) = H_VIEW * (mu_cond_view(:,:,j) / H_VIEW);
    
    mu_cond_view_cartesian(:,j) = [mu_cond_view_back(1:3,4,j);
        rotm2quat(mu_cond_view_back(1:3,1:3))'];
end

% Change viewing frame back for validation
for i = 1:n_sample
    for j = 1:n_step
        g_samples_matrix{i}(:,:,j) = H_VIEW *...
            g_samples_matrix{i}(:,:,j) / H_VIEW;
    end
end
g_samples_view = generate_pose_struct(g_samples_matrix, param.group_name);

% Comparison of mean
disp('Mean difference (total) from original and new views after conditioning')
get_dist_traj(mu_cond, mu_cond_view_back)

%% PLOT: joint PDF
figure; hold on; axis equal;

% World frame
trplot(eye(4), 'rviz', 'notext', 'length', 0.2, 'thick', 1)

% New frame of view
trplot(H_VIEW, 'rviz', 'notext', 'length', 0.2, 'thick', 1)

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:), g_mean.pose(3,:),...
    'b-', 'LineWidth', 1.5)

plot3(g_mean.pose(1,1), g_mean.pose(2,1), g_mean.pose(3,1), 'go')
plot3(g_mean.pose(1,end), g_mean.pose(2,end), g_mean.pose(3,end), 'r*')

% Samples from joint distribution
for i = 1:n_sample
    plot3(g_samples_joint{i}.pose(1,:),...
        g_samples_joint{i}.pose(2,:),...
        g_samples_joint{i}.pose(3,:), 'm-')
    
    plot3(g_samples_joint_view{i}.pose(1,:),...
        g_samples_joint_view{i}.pose(2,:),...
        g_samples_joint_view{i}.pose(3,:), 'c--')
end

%% PLOT: conditional PDF, mean
figure; hold on; axis equal;

% World frame
trplot(eye(4), 'rviz', 'notext', 'length', 0.2, 'thick', 1)

% New frame of view
trplot(H_VIEW, 'rviz', 'notext', 'length', 0.2, 'thick', 1)

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:),...
    g_mean.pose(3,:), 'b-', 'LineWidth', 1.5)

plot3(g_mean.pose(1,1), g_mean.pose(2,1),...
    g_mean.pose(3,1), 'go')
plot3(g_mean.pose(1,end), g_mean.pose(2,end),...
    g_mean.pose(3,end), 'r*')

% Samples after conditioning
trplot(g_via, 'rviz', 'notext', 'length', 0.1, 'thick', 0.1)

% Mean after conditioning
mu_cond_cartesian = nan(7,n_step);
for j = 1:n_step
    mu_cond_cartesian(:,j) = [mu_cond(1:3,4,j);
        rotm2quat(mu_cond(1:3,1:3,j))'];
end
plot3(mu_cond_cartesian(1,:), mu_cond_cartesian(2,:),...
    mu_cond_cartesian(3,:), 'm--', 'LineWidth', 1.5)

% Mean after conditioning, change of view, verify by transforming back
plot3(mu_cond_view_cartesian(1,:), mu_cond_view_cartesian(2,:),...
    mu_cond_view_cartesian(3,:), 'c.-', 'LineWidth', 1.5)

%% PLOT: conditional PDF
figure; hold on; axis equal;

% World frame
trplot(eye(4), 'rviz', 'notext', 'length', 0.2, 'thick', 1)

% New frame of view
trplot(H_VIEW, 'rviz', 'notext', 'length', 0.2, 'thick', 1)

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:),...
    g_mean.pose(3,:), 'b-', 'LineWidth', 1.5)

plot3(g_mean.pose(1,1), g_mean.pose(2,1),...
    g_mean.pose(3,1), 'go')
plot3(g_mean.pose(1,end), g_mean.pose(2,end),...
    g_mean.pose(3,end), 'r*')

% Samples after conditioning
trplot(g_via, 'rviz', 'notext', 'length', 0.1, 'thick', 0.1)

% Mean after conditioning
mu_cond_cartesian = nan(7,n_step);
for j = 1:n_step
    mu_cond_cartesian(:,j) = [mu_cond(1:3,4,j);
        rotm2quat(mu_cond(1:3,1:3,j))'];
end
plot3(mu_cond_cartesian(1,:), mu_cond_cartesian(2,:),...
    mu_cond_cartesian(3,:), 'm--', 'LineWidth', 1.5)

% Mean after conditioning, change of view, verify by transforming back
plot3(mu_cond_view_cartesian(1,:), mu_cond_view_cartesian(2,:),...
    mu_cond_view_cartesian(3,:), 'c--', 'LineWidth', 1.5)

for i = 1:n_sample
    plot3(g_samples{i}.pose(1,:),...
        g_samples{i}.pose(2,:),...
        g_samples{i}.pose(3,:), 'm-')
    
    plot3(g_samples_view{i}.pose(1,:),...
        g_samples_view{i}.pose(2,:),...
        g_samples_view{i}.pose(3,:), 'c-')
    
    %     for j = 1:n_step
    %         trplot(g_samples{i}.matrix(:,:,j), 'rgb', 'notext', 'length', 0.02)
    %     end
end