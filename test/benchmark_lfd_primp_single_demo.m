% Benchmark script for PRIMP learned from a single demonstration
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()

% Name of the dataset
dataset_name = {'panda_arm', 'lasa_handwriting/pose_data'};

for j = 1:length(dataset_name)
    % Name of demo types
    demo_type = load_dataset_param(dataset_name{j});

    for i = 1:length(demo_type)
        disp('Benchmark: PRIMP (ours)')
        disp(['Dataset: ', dataset_name{j}, ' (', num2str(j), '/', num2str(length(dataset_name)), ')'])
        disp(['Demo type: ', demo_type{i}, ' (', num2str(i), '/', num2str(length(demo_type)), ')'])

        % Run benchmark for each demo in each dataset
        run_benchmark(dataset_name{j}, demo_type{i});

        clc;
    end
end

%% Run benchmark for each demo type
function run_benchmark(dataset_name, demo_type)
group_name = {'SE', 'PCG'};
data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");

%% Load and parse demo data
argin.n_step = 50;
argin.data_folder = data_folder;
argin.group_name = 'SE';

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Load random configurations for conditioning
trials = load_random_trials(result_folder);
n_trial = length(trials.t_via{1});

%% Benchmark
res_primp = cell(n_trial, length(group_name));
res_primp_single = cell(n_trial, length(group_name));

for j = 1:length(group_name)
    param.n_sample = 50;
    param.group_name = group_name{j};

    % Compute trajectory distribution from demonstrations
    [g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name{j});

    for i = 1:n_trial
        disp(['Group (', num2str(j), '/', num2str(length(group_name)),...
            '): ', group_name{j}])
        disp([num2str(i/(n_trial) * 100), '%'])

        % Load random via-point poses
        t_via_1 = trials.t_via{1}(i);
        g_via_1 = trials.g_via{1}(:,:,i);
        cov_via_1 = trials.cov_via{1}(:,:,i);

        t_via_2 = trials.t_via{2}(i);
        g_via_2 = trials.g_via{2}(:,:,i);
        cov_via_2 = trials.cov_via{2}(:,:,i);

        % Initiate class
        res_primp{i,j}.group_name = param.group_name;
        res_primp_single{i,j}.group_name = param.group_name;

        t_start = tic;

        primp_obj = PRIMP(g_mean.matrix, cov_t, param);

        % Condition on via-point poses
        primp_obj.get_condition_pdf(t_via_1, g_via_1, cov_via_1);
        primp_obj.get_condition_pdf(t_via_2, g_via_2, cov_via_2);
        g_samples = primp_obj.get_samples();

        t(i,j) = toc(t_start);

        % Use a single random demonstration
        t_start = tic;

        cov_t_single = nan(size(cov_t));
        for k = 1:size(cov_t_single, 3)
            cov_t_single(:,:,k) = 1e-5 * eye(6);
        end

        idx_demo = ceil(length(g_demo) * rand());
        primp_single_obj = PRIMP(g_demo{idx_demo}.matrix, cov_t_single, param);

        % Condition on via-point poses
        primp_single_obj.get_condition_pdf(t_via_1, g_via_1, cov_via_1);
        primp_single_obj.get_condition_pdf(t_via_2, g_via_2, cov_via_2);
        g_samples_single = primp_single_obj.get_samples();

        t_single(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_primp{i,j} =...
            generate_pose_struct(g_samples, param.group_name);
        res_primp_single{i,j} =...
            generate_pose_struct(g_samples_single, param.group_name);

        % Similarity to PRIMP learned from full dataset
        d_sim(i,:,j) =...
            evaluate_traj_distribution(res_primp{i,j}, res_primp_single{i,j});

        % Distance to desired pose
        d_via.full_dataset(i,:,j) =...
            evaluate_desired_pose(res_primp{i,j}, g_via_1, t_via_1);
        d_via.single_demo(i,:,j) =...
            evaluate_desired_pose(res_primp_single{i,j}, g_via_2, t_via_2);
    end

end

%% Evaluation of benchmarks
% Store distance results
res_filename = strcat(result_folder, "result_lfd_primp.mat");
save(res_filename, "t", "t_single", "d_sim", "d_via");

% Display and store command window
diary_filename = strcat(result_folder, "result_lfd_primp.txt");
if exist(diary_filename, 'file') ; delete(diary_filename); end
diary(diary_filename);

for j = 1:length(group_name)
    disp('===============================================================')
    disp(['Group: ', group_name{j}])

    disp('---- Similarity to PRIMP learned from full dataset (rot, tran):')
    disp(num2str( mean(d_sim(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp('>> PRIMP with full dataset')
    disp(num2str( mean(d_via.full_dataset(:,:,j), 1) ))

    disp(">> PRIMP with single demonstration")
    disp(num2str( mean(d_via.single_demo(:,:,j), 1) ))
end

diary off
end