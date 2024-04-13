% Demonstration script for PRIMP
%
%  Data input from "/data/" folder in ".json" format
%  Output 3 files into "/result/" folder:
%    - "reference_density_{demo_type}.json": store mean, joint covariance
%    and covariance for each step
%    - "reference_density_{demo_type}_mean.csv": store mean trajectory in
%    pose format (N x 7 matrix, each row: {translation, orientation}),
%    orientation parameterized by "axis-angle"
%    - "samples_{demo_type}.json": store samples from conditioned
%    probability, in pose format (N x 7 matrix), orientation parameterized
%    by quaternion with order (x, y, z, w)
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
n_sample = 10;

% Name of the dataset
dataset_name = 'panda_arm';
% dataset_name = 'lasa_handwriting/pose_data';

% Type of demonstration
demo_type = "simulation/circle";
% demo_type = "real/trajectory/pouring/default";
% demo_type = "Snake";
% demo_type = "real/trajectory/scooping/default";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = [1e-3 * ones(3,1); 1e-4 * ones(3,1)];
VIA_POSE_SCALE.covariance = 1e-5;

% Indicator of whether to generate random via/goal poses
is_generate_random = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/demo/", dataset_name, "/", demo_type, "/");

%% Load demo data and compute mean/covariance
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate or load random via/goal poses
if is_generate_random
    mkdir(result_folder);

    % Generate random via/goal poses
    trials = generate_random_trials(g_demo{1}, [1, rand()],...
        VIA_POSE_SCALE, result_folder);

    disp("Generated random configurations!")
else
    % Load random configurations for conditioning
    trials = load_random_trials(result_folder);
    n_trial = length(trials.t_via{1});

    disp("Loaded randomly generated configurations!");
end

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
primp_obj = PRIMP(g_mean.matrix, cov_t, param);

tic;
[mu_cond_1, sigma_cond_1] = primp_obj.get_condition_pdf(1.0,...
    g_goal, cov_goal);
g_samples_1 = primp_obj.get_samples();

[mu_cond_2, sigma_cond_2] = primp_obj.get_condition_pdf(t_via,...
    g_via, cov_via);
g_samples_2 = primp_obj.get_samples();
toc;

% Convert to pose
pose_samples_1 = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples_1{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples_1{i}(j,:) = homo2pose_quat(g_samples_1{i}(:,:,j));
    end
end

pose_samples_2 = cell(n_sample, 1);
for i = 1:n_sample
    pose_samples_2{i} = nan(n_step, 7);
    for j = 1:n_step
        pose_samples_2{i}(j,:) = homo2pose_quat(g_samples_2{i}(:,:,j));
    end
end

% Mean and covariance after conditioning
pose_cond_mean_1 = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean_1(j,:) = homo2pose_axang(mu_cond_1(:,:,j));
end

pose_cond_mean_2 = nan(n_step, 7);
for j = 1:n_step
    pose_cond_mean_2(j,:) = homo2pose_axang(mu_cond_2(:,:,j));
end

sigma_cond_step = zeros(6, 6, n_step);
sigma_cond_step(:,:,1) = cov_t(:,:,1);
for i = 2:n_step-1
    idx_block = 6*(i-2)+1:6*(i-1);
    sigma_cond_step(:,:,i) = sigma_cond_2(idx_block, idx_block);
end
sigma_cond_step(:,:,end) = cov_goal;

var_cond_step = zeros(n_step, 6);
for i = 1:n_step
    var_cond_step(i,:) = diag(sqrt(sigma_cond_step(:,:,i)));
end

%% Store distribution and samples from conditional probability
% Mean and covariance
prob_data.num_steps = n_step;
prob_data.mean = permute(mu_cond_2, [3,1,2]);
prob_data.covariance_joint = sigma_cond_1;
prob_data.covariance_step = permute(sigma_cond_step, [3,1,2]);

result_prefix = strcat(result_folder, 'primp_', group_name, '/');
mkdir(result_prefix);

json_data = jsonencode(prob_data);
fid = fopen(strcat(result_prefix, 'reference_density.json'), 'w');
fprintf(fid, '%s', json_data);
fclose(fid);

writematrix(pose_cond_mean_1, strcat(result_prefix, 'reference_density_mean.csv'));

% Samples
sample_data.num_samples = n_sample;
sample_data.num_steps = n_step;
sample_data.samples = pose_samples_1;

json_data = jsonencode(sample_data);
fid = fopen(strcat(result_prefix, 'samples_.json'), 'w');
fprintf(fid, '%s', json_data);
fclose(fid);

%% PLOT: Demonstrations
figure; hold on; axis off; axis equal;

% Demos
for i = 1:n_demo
    plot3(g_demo{i}.pose(1,:), g_demo{i}.pose(2,:), g_demo{i}.pose(3,:), 'c')
end

% Prior mean poses
pose_mean = g_mean.pose;
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 1.5)
% plot3(pose_mean(1,1), pose_mean(2,1), pose_mean(3,1), 'go')
% plot3(pose_mean(1,end), pose_mean(2,end), pose_mean(3,end), 'r*')

% for j = 1:n_step
%     trplot(g_mean.matrix(:,:,j), 'rgb', 'notext', 'length', 0.01)
% end

%% PLOT: Condition on goal pose
frame_scale = 0.1;

figure; hold on; axis off; axis equal;
% Prior mean poses
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 3)
% plot3(pose_mean(1,1), pose_mean(2,1), pose_mean(3,1), 'go')
% plot3(pose_mean(1,end), pose_mean(2,end), pose_mean(3,end), 'r*')

% Samples after conditioning
trplot(g_goal, 'rviz', 'notext', 'length', frame_scale, 'width', 1)

for i = 1:n_sample
    plot3(pose_samples_1{i}(:,1), pose_samples_1{i}(:,2),...
        pose_samples_1{i}(:,3), 'm--', 'LineWidth', 1)

    plot3(pose_cond_mean_1(:,1), pose_cond_mean_1(:,2),...
        pose_cond_mean_1(:,3), 'm-', 'LineWidth', 3)

    %     for j = 1:2:n_step
    %         trplot(mu_cond_1(:,:,j), 'rgb', 'notext', 'length', 0.05)
    %     end
end

%% PLOT: Condition on goal and a via pose
figure; hold on; axis off; axis equal;
% Prior mean poses
plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
    'b-', 'LineWidth', 3)
% plot3(pose_mean(1,1), pose_mean(2,1), pose_mean(3,1), 'go')
% plot3(pose_mean(1,end), pose_mean(2,end), pose_mean(3,end), 'r*')

% Samples after condition on via pose
trplot(g_goal, 'rviz', 'notext', 'length', frame_scale, 'width', 1)
trplot(g_via, 'rviz', 'notext', 'length', frame_scale, 'width', 1)

plot3(pose_cond_mean_2(:,1), pose_cond_mean_2(:,2),...
    pose_cond_mean_2(:,3), 'm-', 'LineWidth', 3)

%     for j = 1:2:n_step
%         trplot(mu_cond_2(:,:,j), 'rgb', 'notext', 'length', 0.05)
%     end

for i = 1:n_sample
    plot3(pose_samples_2{i}(:,1), pose_samples_2{i}(:,2),...
        pose_samples_2{i}(:,3), 'm--', 'LineWidth', 1)
end

%% PLOT: Trajectory profile
figure;

% Convert to Lie algebra
t_steps = 0:1/(n_step-1):1;
exp_via = get_exp_coord(g_via, argin.group_name);
exp_goal = get_exp_coord(g_goal, argin.group_name);

primp_mean_via = zeros(n_step, 6);
for i = 1:n_step
    primp_mean_via(i,:) = get_exp_coord(mu_cond_2(:,:,i), argin.group_name);
end

% For translation part
subplot(2,1,1); grid on; hold on;

plot(t_via, exp_via(4), 'o', t_via, exp_via(5), 'o',...
    t_via, exp_via(6), 'o')
plot(1, exp_goal(4), '*', 1, exp_goal(5), '*', 1, exp_goal(6), '*')
plot(t_steps, primp_mean_via(:,4), t_steps, primp_mean_via(:,5),...
    t_steps, primp_mean_via(:,6), 'LineWidth', 3)

plot(t_steps, primp_mean_via(:,4) + var_cond_step(:,4), 'm--',...
    t_steps, primp_mean_via(:,5) + var_cond_step(:,5), 'm--',...
    t_steps, primp_mean_via(:,6) + var_cond_step(:,6), 'm--',...
    'LineWidth', 1.5)
plot(t_steps, primp_mean_via(:,4) - var_cond_step(:,4), 'm--',...
    t_steps, primp_mean_via(:,5) - var_cond_step(:,5), 'm--',...
    t_steps, primp_mean_via(:,6) - var_cond_step(:,6), 'm--',...
    'LineWidth', 1.5)

title('Translation part')
xlabel('Time')

% For rotation part, in exponential coordinates
subplot(2,1,2); grid on; hold on;

plot(t_via, exp_via(1), 'o', t_via, exp_via(2), 'o',...
    t_via, exp_via(3), 'o')
plot(1, exp_goal(1), '*', 1, exp_goal(2), '*', 1, exp_goal(3), '*')
plot(t_steps, primp_mean_via(:,1), t_steps, primp_mean_via(:,2),...
    t_steps, primp_mean_via(:,3), 'LineWidth', 3)

plot(t_steps, primp_mean_via(:,1) + var_cond_step(:,1), 'm--',...
    t_steps, primp_mean_via(:,2) + var_cond_step(:,2), 'm--',...
    t_steps, primp_mean_via(:,3) + var_cond_step(:,3), 'm--',...
    'LineWidth', 1.5)
plot(t_steps, primp_mean_via(:,1) - var_cond_step(:,1), 'm--',...
    t_steps, primp_mean_via(:,2) - var_cond_step(:,2), 'm--',...
    t_steps, primp_mean_via(:,3) - var_cond_step(:,3), 'm--',...
    'LineWidth', 1.5)

title('Rotation part, in Lie algebra so(3)')
xlabel('Time')