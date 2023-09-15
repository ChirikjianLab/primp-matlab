% Benchmark script for PRIMP
%
%  Author
%    Sipu Ruan, 2023
close all; clear; clc;
add_paths()

% Name of the dataset
dataset_name = 'panda_arm';
% dataset_name = 'lasa_handwriting/pose_data';

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

% Group name
group_name = {'SE', 'PCG'};

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = 1;
VIA_POSE_SCALE.covariance = 1e-4;

% Indicator of whether to generate random via/goal poses
is_generate_random = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");

mkdir(result_folder);

%% Load and parse demo data
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = 'SE';

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

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

%% Benchmark
res_goal = cell(n_trial, length(group_name));
res_via = cell(n_trial, length(group_name));

for j = 1:length(group_name)
    % Compute trajectory distribution from demonstrations
    [g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name{j});

    for i = 1:n_trial
        clc;
        disp('Benchmark: PRIMP (ours)')
        disp(['Dataset: ', dataset_name])
        disp(['Demo type: ', demo_type])
        disp(['Group (', num2str(j), '/', num2str(length(group_name)),...
            '): ', group_name{j}])
        disp([num2str(i/(n_trial) * 100), '%'])

        % Load random via/goal poses
        g_goal = trials.g_via{1}(:,:,i);
        cov_goal = trials.cov_via{1}(:,:,i);

        t_via = trials.t_via{2}(i);
        g_via = trials.g_via{2}(:,:,i);
        cov_via = trials.cov_via{2}(:,:,i);

        % Initiate class
        param.n_sample = n_sample;
        param.group_name = group_name{j};

        res_goal{i,j}.group_name = param.group_name;
        res_via{i,j}.group_name = param.group_name;

        t_start = tic;

        primp_obj = PRIMP(g_mean.matrix, cov_t, param);

        % Condition on goal pose
        primp_obj.get_condition_pdf(1.0, g_goal, cov_goal);
        g_samples_goal = primp_obj.get_samples();

        % Condition on a via pose
        primp_obj.get_condition_pdf(t_via, g_via, cov_via);
        g_samples_via = primp_obj.get_samples();

        t(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_goal{i,j} =...
            generate_pose_struct(g_samples_goal, param.group_name);
        res_via{i,j} =...
            generate_pose_struct(g_samples_via, param.group_name);

        % Distance to demonstrated trajectories
        d_demo.goal(i,:,j) =...
            evaluate_traj_distribution(res_goal{i,j}, g_demo);
        d_demo.via(i,:,j) =...
            evaluate_traj_distribution(res_via{i,j}, g_demo);

        % Distance to desired pose
        d_via.goal(i,:,j) =...
            evaluate_desired_pose(res_goal{i,j}, g_goal, 1);
        d_via.via(i,:,j) =...
            evaluate_desired_pose(res_via{i,j}, g_via, t_via);
    end

end

%% Evaluation of benchmarks
% Store distance results
res_filename = strcat(result_folder, "result_lfd_primp.mat");
save(res_filename, "t", "d_demo", "d_via");

% Display and store command window
diary_filename = strcat(result_folder, "result_lfd_primp.txt");
if exist(diary_filename, 'file') ; delete(diary_filename); end
diary(diary_filename);

for j = 1:length(group_name)
    disp('===============================================================')
    disp(['Group: ', group_name{j}])

    disp('>>>> Condition on goal <<<<')
    disp('---- Distance to demo (rot, tran):')
    disp(num2str( mean(d_demo.goal(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp(num2str( mean(d_via.goal(:,:,j), 1) ))

    disp('---------------------------------------------------------------')

    disp('>>>> Condition on goal and a via pose <<<<')
    disp('---- Distance to demo (rot, tran):')
    disp(num2str( mean(d_demo.via(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp(num2str( mean(d_via.via(:,:,j), 1) ))
end

% Computational time
figure; hold on;

disp('>>>> Computation time <<<<')
disp(num2str( mean(t,1) ))
boxplot(t)
ylim([0, max(max(t))])

diary off
end