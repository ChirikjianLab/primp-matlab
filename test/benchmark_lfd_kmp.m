% Benchmark script for Kernelized Movement Primitives
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()
addpath ../../../Toolbox/learn_from_demo/pbdlib-matlab/demos/m_fcts/
addpath ../../../Toolbox/learn_from_demo/robInfLib-matlab/fcts/

% Name of the dataset
dataset_name = {'panda_arm', 'lasa_handwriting/pose_data'};

for j = 1:length(dataset_name)
    % Name of demo types
    demo_type = load_dataset_param(dataset_name{j});

    for i = 1:length(demo_type)
        disp('Benchmark: Kernalized Movement Primitives')
        disp(['Dataset: ', dataset_name{j}, ' (', num2str(j), '/', num2str(length(dataset_name)), ')'])
        disp(['Demo type: ', demo_type{i}, ' (', num2str(i), '/', num2str(length(demo_type)), ')'])

        % Run benchmark for each demo type
        run_benchmark(dataset_name{j}, demo_type{i});

        clc;
    end
end

function run_benchmark(dataset_name, demo_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of samples on distribution
n_sample = 50;

% Number of time steps
n_step = 50;

% Number of states in the GMM
n_state = 8;

% KMP parameters
lamda = 1e-2;  % control mean prediction
lamdac = 60; % control variance prediction
kh = [0.1, 1, 10]; % Scale of Gaussian kernel basis
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
argin.group_name = 'SE';

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Load random configurations for conditioning
trials = load_random_trials(result_folder);
n_trial = length(trials.t_via{1});

%% Benchmark
res_kmp_via_1 = cell(n_trial, 1);
res_kmp_via_2 = cell(n_trial, 1);

for j = 1:length(kh)
    param.kmp_param.kh = kh(j);

    for i = 1:n_trial
        disp(['Regularization: ', num2str(param.kmp_param.lamda)]); 
        disp(['Kernel scale: ', num2str(param.kmp_param.kh)]);
        disp([num2str(i/(n_trial) * 100), '%'])

        % Load random via-point poses
        t_via_1 = trials.t_via{1}(i);
        g_via_1 = trials.g_via{1}(:,:,i);
        cov_via_1 = trials.cov_via{1}(:,:,i);

        t_via_2 = trials.t_via{2}(i);
        g_via_2 = trials.g_via{2}(:,:,i);
        cov_via_2 = trials.cov_via{2}(:,:,i);

        %%%%%%%%%%%%%%%%%%%%%% Using KMP method %%%%%%%%%%%%%%%%%%%%%%%%%%%
        t_start = tic;

        kmp_obj = kmp(g_demo, model, param);

        % Condition on goal pose
        kmp_obj.compute_kmp_via_point(g_via_1, cov_via_1, t_via_1);
        traj_1 = kmp_obj.get_kmp_trajectory();
        g_samples_1 = kmp_obj.get_samples(traj_1, n_sample);

        % Condition on via pose
        kmp_obj.compute_kmp_via_point(g_via_2, cov_via_2, t_via_2);
        traj_2 = kmp_obj.get_kmp_trajectory();
        g_samples_2 = kmp_obj.get_samples(traj_2, n_sample);

        t_kmp(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_kmp_via_1{i,j} =...
            generate_pose_struct(g_samples_1, argin.group_name);
        res_kmp_via_2{i,j} =...
            generate_pose_struct(g_samples_2, argin.group_name);

        % Distance to demonstrated trajectories
        d_demo_kmp.via_1(i,:,j) =...
            evaluate_traj_distribution(res_kmp_via_1{i,j}, g_demo);
        d_demo_kmp.via_2(i,:,j) =...
            evaluate_traj_distribution(res_kmp_via_2{i,j}, g_demo);

        % Distance to desired pose
        d_via_kmp.via_1(i,:,j) =...
            evaluate_desired_pose(res_kmp_via_1{i,j}, g_via_1, t_via_1);
        d_via_kmp.via_2(i,:,j) =...
            evaluate_desired_pose(res_kmp_via_2{i,j}, g_via_2, t_via_2);
    end
end

%% Evaluation of benchmarks
result_filename = "result_lfd_kmp_lamda_1e-2";

% Store distance results
res_filename = strcat(result_folder, result_filename, ".mat");
save(res_filename, "t_kmp", "d_demo_kmp", "d_via_kmp", "res_kmp_via_1",...
    "res_kmp_via_2", "n_state", "param");

% Display and store command window
diary_filename = strcat(result_folder, result_filename, ".txt");
if exist(diary_filename, 'file') ; delete(diary_filename); end
diary(diary_filename);

disp('===============================================================')
disp('>>>> Condition on 1 via point <<<<')
disp('---- Distance to demo (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_demo_kmp.via_1(:,:,j), 1) )])
end

disp('---- Distance to desired pose (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_via_kmp.via_1(:,:,j), 1) )])
end

disp('---------------------------------------------------------------')

disp('>>>> Condition on 2 via points <<<<')
disp('---- Distance to demo (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_demo_kmp.via_2(:,:,j), 1) )])
end

disp('---- Distance to desired pose (rot, tran):')
for j = 1:length(kh)
    disp(['KMP method: (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(d_via_kmp.via_2(:,:,j), 1) )])
end

% Computational time
disp('>>>> Computation time <<<<')
for j = 1:length(kh)
    disp(['KMP method: (kh = ', num2str(kh(j)), '): ',...
        num2str( mean(t_kmp(:,j)) )])
end

diary off
end