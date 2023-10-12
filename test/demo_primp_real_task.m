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
param.n_sample = 5;

% Type of demonstration
% demo_type = "pouring/default";
demo_type = "transporting/default";
% demo_type = "scooping/default";

% Group name: 'SE', 'PCG'
param.group_name = 'PCG';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real';

% Data and result folder
param.data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");
traj_folder = strcat(result_folder, "/primp_", param.group_name);
mkdir(traj_folder);

% Load demonstrations
filenames = dir(strcat(param.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, param);

% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, param.group_name);
n_demo = length(g_demo);

% Load random configurations for conditioning
trials = load_random_trials(result_folder, 2);
n_trial = length(trials.t_via{1});

try
    pose_sim2real = load(strcat(result_folder, "/sim2real_transform.csv"));
    g_sim2real = [quat2rotm(pose_sim2real(1,4:end)), pose_sim2real(1,1:3)';
        0, 0, 0, 1];
catch
end

%% PRIMP main routine
pose_cond_mean_tool = nan(param.n_step, 7, n_trial);

idx = ceil(n_trial * rand);

clc;
disp(strcat(num2str(idx), "/", num2str(n_trial)));

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

try
    % Transform to tool frame
    mu_cond_tool = nan(4, 4, param.n_step);
    for j = 1:param.n_step
        mu_cond_tool(:,:,j) = mu_cond(:,:,j) / g_sim2real;
        pose_cond_mean_tool(j,:,idx) = homo2pose_quat(mu_cond_tool(:,:,j));
    end

    %% Save trajectories
    trajectory_primp.num_trials = n_trial;
    trajectory_primp.pose_format = "[x,y,z,qx,qy,qz,qw]";
    trajectory_primp.tool_trajectory = permute(pose_cond_mean_tool, [3,1,2]);

    json_data = jsonencode(trajectory_primp);
    fid = fopen( strcat(result_folder, 'trajectory_primp.json'), 'w');
    fprintf(fid, '%s', json_data);
    fclose(fid);

    disp("Tool trajectories saved to file!");

catch
end

%% PLOT: Condition on via-point poses
frame_scale = 0.1;

figure; hold on; axis off; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:), 'c')
end

% Prior mean poses
plot3(g_mean.pose(1,:), g_mean.pose(2,:), g_mean.pose(3,:),...
    'b-', 'LineWidth', 3)

% Samples after condition on via pose
for i = 1:length(trials.g_via)
    trplot(trials.g_via{i}(:,:,idx), 'rviz', 'notext', 'length', frame_scale, 'width', 1)
end

pose_cond_mean = nan(param.n_step, 7);
for m = 1:param.n_step
    pose_cond_mean(m,:) = homo2pose_axang(mu_cond(:,:,m));
end

plot3(pose_cond_mean(:,1), pose_cond_mean(:,2),...
    pose_cond_mean(:,3), 'm-', 'LineWidth', 3)

% for j = 1:2:n_step
%     trplot(mu_cond(:,:,j), 'rgb', 'notext', 'length', 0.05)
% end

% pose_samples = generate_pose_struct(g_samples, param.group_name);
% for i = 1:param.n_sample
%     plot3(pose_samples{i}.pose(1,:), pose_samples{i}.pose(2,:),...
%         pose_samples{i}.pose(3,:), 'm--', 'LineWidth', 1)
% end

try
    % Tool frames after conditioning
    for j = 1:2:param.n_step
        trplot(mu_cond_tool(:,:,j), 'rgb', 'notext', 'length', 0.05)
    end
catch
end

%% PLOT: add robot model
figure;

robot = loadrobot("frankaEmikaPanda");

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];
init = robot.homeConfiguration;

for i = 1:param.n_step
    [configSol{i}, ~] = ik('panda_link8', mu_cond(:,:,i), weights, init);
end
robot.show(configSol{1}, 'Frames', 'off');
hold on; axis equal; axis off;
robot.show(configSol{end}, 'Frames', 'off');

for i = 1:length(trials.g_via)
    trplot(trials.g_via{i}(:,:,idx), 'rviz', 'notext', 'length', frame_scale, 'width', 1)
end

plot3(pose_cond_mean(:,1), pose_cond_mean(:,2),...
    pose_cond_mean(:,3), 'm-', 'LineWidth', 3)

try
    for j = 1:2:param.n_step
        trplot(mu_cond_tool(:,:,j), 'rgb', 'notext', 'length', 0.05)
    end
catch
end