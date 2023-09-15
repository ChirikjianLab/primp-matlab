% Benchmark script for Orientation-KMP
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()
addpath ../../../Toolbox/learn_from_demo/pbdlib-matlab/demos/m_fcts/
addpath ../../../Toolbox/learn_from_demo/robInfLib-matlab/fcts/

% Name of the dataset
dataset_name = 'panda_arm';
% dataset_name = 'lasa_handriting/pose_data';

demo_type = load_dataset_param(dataset_name);

%% Run benchmark for each demo type
for i = 1:length(demo_type)
    run_benchmark(dataset_name, demo_type{i});
end

function run_benchmark(dataset_name, demo_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of benchmark trials
n_trial = 50;

% Number of samples on distribution
n_sample = 50;

% Number of time steps
n_step = 50;

% Number of states in the GMM
n_state = 8;

% KMP parameters
lamda = 1;  % control mean prediction
lamdac = 60; % control variance prediction
kh = [0.1, 1, 10]; % Scale of Gaussian kernel basis

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 1;
VIA_POSE_SCALE.covariance = 1e-4;

% Indicator of whether to generate random via/goal poses
is_generate_random = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");

% Parameters
param.n_step = n_step;
param.dim = 3;
param.kmp_param.lamda = lamda;
param.kmp_param.lamdac = lamdac;
model.nbStates = n_state;

%% Load and parse demo data
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = 'PCG';

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate or load random via/goal poses
if is_generate_random
    mkdir(result_folder);

    % Generate random via/goal poses
    trials = generate_random_trials(g_demo{1}, n_trial,...
        VIA_POSE_SCALE, result_folder);

    disp("Generated random configurations!")
else
    % Load random configurations for conditioning
    trials = load_random_trials(result_folder);
    n_trial = length(trials.t_via{1});

    disp("Loaded randomly generated configurations!");
end

%% Benchmark
res_kmp_goal = cell(n_trial, 1);
res_kmp_via = cell(n_trial, 1);

for j = 1:length(kh)

    param.kmp_param.kh = kh(j);

    for i = 1:n_trial
        clc;
        disp('Benchmark: Kernalized Movement Primitives')
        disp(['Dataset: ', dataset_name])
        disp(['Demo type: ', demo_type])
        disp(['Regularization: ', num2str(param.kmp_param.lamda)]); 
        disp(['Kernel scale: ', num2str(param.kmp_param.kh)]);
        disp([num2str(i/(n_trial) * 100), '%'])

        % Load random via/goal poses
        g_goal = trials.g_via{1}(:,:,i);
        cov_goal = trials.cov_via{1}(:,:,i);

        t_via = trials.t_via{2}(i);
        g_via = trials.g_via{2}(:,:,i);
        cov_via = trials.cov_via{2}(:,:,i);

        %%%%%%%%%%%%%%%%%%%%%% Using KMP method %%%%%%%%%%%%%%%%%%%%%%%%%%%
        t_start = tic;

        kmp_obj = kmp(g_demo, model, param);

        % Condition on goal pose
        kmp_obj.compute_kmp_via_point(g_goal, cov_goal, 1.0);
        traj_kmp_goal = kmp_obj.get_kmp_trajectory();
        g_samples_kmp_goal = kmp_obj.get_samples(traj_kmp_goal, n_sample);

        % Condition on via pose
        kmp_obj.compute_kmp_via_point(g_via, cov_via, t_via);
        traj_kmp_via = kmp_obj.get_kmp_trajectory();
        g_samples_kmp_via = kmp_obj.get_samples(traj_kmp_via, n_sample);

        t_kmp(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_kmp_goal{i,j} =...
            generate_pose_struct(g_samples_kmp_goal, argin.group_name);
        res_kmp_via{i,j} =...
            generate_pose_struct(g_samples_kmp_via, argin.group_name);

        % Distance to demonstrated trajectories
        d_demo_kmp.goal(i,:,j) =...
            evaluate_traj_distribution(res_kmp_goal{i,j}, g_demo);
        d_demo_kmp.via(i,:,j) =...
            evaluate_traj_distribution(res_kmp_via{i,j}, g_demo);

        % Distance to desired pose
        d_via_kmp.goal(i,:,j) =...
            evaluate_desired_pose(res_kmp_goal{i,j}, g_goal, 1);
        d_via_kmp.via(i,:,j) =...
            evaluate_desired_pose(res_kmp_via{i,j}, g_via, t_via);
    end
end

%% Evaluation of benchmarks
result_filename = "result_lfd_kmp";

% Store distance results
res_filename = strcat(result_folder, result_filename, ".mat");
save(res_filename, "t_kmp", "d_demo_kmp", "d_via_kmp",...
    "n_state", "param");

% Display and store command window
diary_filename = strcat(result_folder, result_filename, ".txt");
if exist(diary_filename, 'file') ; delete(diary_filename); end
diary(diary_filename);

disp('===============================================================')
disp('>>>> Condition on goal <<<<')
disp('---- Distance to demo (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_demo_kmp.goal(:,:,j), 1) )])
end

disp('---- Distance to desired pose (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_via_kmp.goal(:,:,j), 1) )])
end

disp('---------------------------------------------------------------')

disp('>>>> Condition on goal and a via pose <<<<')
disp('---- Distance to demo (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_demo_kmp.via(:,:,j), 1) )])
end

disp('---- Distance to desired pose (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method: (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_via_kmp.via(:,:,j), 1) )])
end

% Computational time
disp('>>>> Computation time <<<<')
for j = 1:length(kh)
    disp(['KMP method: (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(t_kmp(:,j)) )])
end

diary off
end