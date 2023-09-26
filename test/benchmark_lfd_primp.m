% Benchmark script for PRIMP
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
res_via_1 = cell(n_trial, length(group_name));
res_via_2 = cell(n_trial, length(group_name));

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
        res_via_1{i,j}.group_name = param.group_name;
        res_via_2{i,j}.group_name = param.group_name;

        t_start = tic;

        primp_obj = PRIMP(g_mean.matrix, cov_t, param);

        % Condition on via-point poses
        primp_obj.get_condition_pdf(t_via_1, g_via_1, cov_via_1);
        g_samples_1 = primp_obj.get_samples();

        primp_obj.get_condition_pdf(t_via_2, g_via_2, cov_via_2);
        g_samples_2 = primp_obj.get_samples();

        t(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_via_1{i,j} =...
            generate_pose_struct(g_samples_1, param.group_name);
        res_via_2{i,j} =...
            generate_pose_struct(g_samples_2, param.group_name);

        % Distance to demonstrated trajectories
        d_demo.via_1(i,:,j) =...
            evaluate_traj_distribution(res_via_1{i,j}, g_demo);
        d_demo.via_2(i,:,j) =...
            evaluate_traj_distribution(res_via_2{i,j}, g_demo);

        % Distance to desired pose
        d_via.via_1(i,:,j) =...
            evaluate_desired_pose(res_via_1{i,j}, g_via_1, t_via_1);
        d_via.via_2(i,:,j) =...
            evaluate_desired_pose(res_via_2{i,j}, g_via_2, t_via_2);
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

    disp('>>>> Condition on 1 via point <<<<')
    disp('---- Distance to demo (rot, tran):')
    disp(num2str( mean(d_demo.via_1(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp(num2str( mean(d_via.via_1(:,:,j), 1) ))

    disp('---------------------------------------------------------------')

    disp('>>>> Condition on 2 via points <<<<')
    disp('---- Distance to demo (rot, tran):')
    disp(num2str( mean(d_demo.via_2(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp(num2str( mean(d_via.via_2(:,:,j), 1) ))
end

% Computational time
figure; hold on;

disp('>>>> Computation time <<<<')
disp(num2str( mean(t,1) ))
boxplot(t)
ylim([0, max(max(t))])

diary off
end