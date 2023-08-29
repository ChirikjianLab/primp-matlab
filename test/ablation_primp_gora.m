% Script for ablation study on GORA as a pre-process for PRIMP
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()

% Name of the dataset
% dataset_name = 'panda_arm';
dataset_name = 'lasa_handwriting/pose_data';

demo_type = load_dataset_param(dataset_name);

%% Run ablations for each demo type
for i = 1:length(demo_type)
    run_ablation(dataset_name, demo_type{i});
end

function run_ablation(dataset_name, demo_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of trials for the ablation study
n_trial = 50;

% Number of samples on distribution
n_sample = 5;

% Number of time steps
n_step = 50;

% Group name
group_name = 'SE';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 1;
VIA_POSE_SCALE.covariance = 1e-4;

% Indicator of whether to generate random via/goal poses
is_generate_random = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/ablation/gora/", dataset_name, "/",...
    demo_type, "/");
result_prefix = strcat(result_folder, 'primp_', group_name, '/');

mkdir(result_folder);
mkdir(result_prefix);

%% Load and parse demo data
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = group_name;

filenames = dir(strcat(argin.data_folder, "*.json"));

% Parse demo trajectories
% with GORA
argin.align_method = 'gora';
g_demo = parse_demo_trajectory(filenames, argin);
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);

% without GORA
argin.align_method = 'interp';
g_demo_no_gora = parse_demo_trajectory(filenames, argin);
[g_mean_no_gora, cov_t_no_gora] = get_pdf_from_demo(g_demo_no_gora,...
    group_name);

% with original DTW
argin.align_method = 'dtw';
g_demo_no_gora = parse_demo_trajectory(filenames, argin);
[g_mean_dtw, cov_t_dtw] = get_pdf_from_demo(g_demo_no_gora,...
    group_name);

% Generate or load random via/goal poses
if is_generate_random
    mkdir(result_folder);

    % Generate random via/goal poses
    trials = generate_random_trials(g_demo{1}, n_trial, VIA_POSE_SCALE,...
        result_folder);

    disp("Generated random configurations!")
else
    % Load random configurations for conditioning
    trials = load_random_trials(result_folder);
    n_trial = length(trials.t_via{1});

    disp("Loaded randomly generated configurations!");
end

%% Ablation
res_primp = cell(n_trial, 1);
res_primp_no_gora = cell(n_trial, 1);
res_primp_dtw = cell(n_trial, 1);

for i = 1:n_trial
    clc;
    disp('Ablation study for GORA subroutine in PRIMP')
    disp(['Dataset: ', dataset_name])
    disp(['Demo type: ', demo_type])
    disp(['Group: ', group_name])
    disp([num2str(i/(n_trial) * 100), '%'])

    % Load random via/goal poses
    g_goal = trials.g_via{1}(:,:,i);
    cov_goal = trials.cov_via{1}(:,:,i);

    t_via = trials.t_via{2}(i);
    g_via = trials.g_via{2}(:,:,i);
    cov_via = trials.cov_via{2}(:,:,i);

    % Initiate class
    param.n_sample = n_sample;
    param.group_name = group_name;

    res_primp{i}.group_name = param.group_name;
    res_primp_no_gora{i}.group_name = param.group_name;
    res_primp_dtw{i}.group_name = param.group_name;

    %% Main routine
    primp_obj = PRIMP(g_mean.matrix, cov_t, param);
    primp_no_gora_obj = PRIMP(g_mean_no_gora.matrix, cov_t_no_gora, param);
    primp_dtw_obj = PRIMP(g_mean_dtw.matrix, cov_t_dtw, param);

    % PRIMP
    t_start = tic;
    primp_obj.get_condition_pdf(1.0, g_goal, cov_goal);
    primp_obj.get_condition_pdf(t_via, g_via, cov_via);
    g_samples = primp_obj.get_samples();
    metric_primp.time(i,1) = toc(t_start);

    % PRIMP without GORA (ablated)
    t_start = tic;
    primp_no_gora_obj.get_condition_pdf(1.0, g_goal, cov_goal);
    primp_no_gora_obj.get_condition_pdf(t_via, g_via, cov_via);
    g_samples_no_gora = primp_no_gora_obj.get_samples();
    metric_primp_no_gora.time(i,1) = toc(t_start);

    % PRIMP with DTW
    t_start = tic;
    primp_dtw_obj.get_condition_pdf(1.0, g_goal, cov_goal);
    primp_dtw_obj.get_condition_pdf(t_via, g_via, cov_via);
    g_samples_dtw = primp_dtw_obj.get_samples();
    metric_primp_dtw.time(i,1) = toc(t_start);

    %% Distance to desired pose and original trajectory
    % Convert to group structure
    res_primp{i} =...
        generate_pose_struct(g_samples, param.group_name);
    res_primp_no_gora{i} =...
        generate_pose_struct(g_samples_no_gora, param.group_name);
    res_primp_dtw{i} =...
        generate_pose_struct(g_samples_dtw, param.group_name);

    % Distance to demonstrated trajectories
    metric_primp.d_demo(i,:) =...
        evaluate_traj_distribution(res_primp{i}, g_demo);
    metric_primp_no_gora.d_demo(i,:) =...
        evaluate_traj_distribution(res_primp_no_gora{i}, g_demo);
    metric_primp_dtw.d_demo(i,:) =...
        evaluate_traj_distribution(res_primp_dtw{i}, g_demo);

    % Distance to desired pose
    metric_primp.d_via(i,:) =...
        evaluate_desired_pose(res_primp{i}, g_goal, 1);
    metric_primp_no_gora.d_via(i,:) =...
        evaluate_desired_pose(res_primp_no_gora{i}, g_via, t_via);
    metric_primp_dtw.d_via(i,:) =...
        evaluate_desired_pose(res_primp_dtw{i}, g_via, t_via);
end

%% Evaluation of ablation study
% Store results as .mat file
res_filename = strcat(result_folder, "result_ablation_primp_gora.mat");
save(res_filename, "res_primp", "res_primp_no_gora", "res_primp_dtw",...
    "metric_primp", "metric_primp_no_gora", "metric_primp_dtw");

% Display and store command window
diary_filename = strcat(result_folder, "result_ablation_primp_gora.txt");
diary(diary_filename);

disp('===============================================================')
disp('Ablation results for GORA subroutine in PRIMP')
disp(['Group: ', group_name])
disp('===============================================================')

disp('>>>> PRIMP <<<<')
disp('---- Distance to demo (rot, tran):')
disp(num2str( mean(metric_primp.d_demo, 1) ))

disp('---- Distance to desired pose (rot, tran):')
disp(num2str( mean(metric_primp.d_via, 1) ))

disp('---- Averaged computation time ----')
disp(num2str( mean(metric_primp.time, 1) ))

disp('---------------------------------------------------------------')

disp('>>>> PRIMP without GORA (ablated) <<<<')
disp('---- Distance to demo (rot, tran):')
disp(num2str( mean(metric_primp_no_gora.d_demo, 1) ))

disp('---- Distance to desired pose (rot, tran):')
disp(num2str( mean(metric_primp_no_gora.d_via, 1) ))

disp('---- Averaged computation time ----')
disp(num2str( mean(metric_primp_no_gora.time, 1) ))

disp('---------------------------------------------------------------')

disp('>>>> PRIMP with DTW <<<<')
disp('---- Distance to demo (rot, tran):')
disp(num2str( mean(metric_primp_dtw.d_demo, 1) ))

disp('---- Distance to desired pose (rot, tran):')
disp(num2str( mean(metric_primp_dtw.d_via, 1) ))

disp('---- Averaged computation time ----')
disp(num2str( mean(metric_primp_dtw.time, 1) ))

% Computational time
figure; hold on;
t = [metric_primp.time, metric_primp_no_gora.time, metric_primp_dtw.time];
boxplot(t)
ylim([0, max(max(t))])

diary off

end