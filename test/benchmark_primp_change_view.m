% Benchmark script for PRIMP in adapting to the change of viewing frames
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
cov_via = 1e-5*eye(6);

%% Benchmark
h_view = cell(n_trial, length(group_name));
res_primp = cell(n_trial, length(group_name));
res_primp_no_equi = cell(n_trial, length(group_name));

for j = 1:length(group_name)
    param.n_sample = 50;
    param.group_name = group_name{j};

    % Compute trajectory distribution from demonstrations
    [g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name{j});

    for i = 1:n_trial
        disp(['Group (', num2str(j), '/', num2str(length(group_name)),...
            '): ', group_name{j}])
        disp([num2str(i/(n_trial) * 100), '%'])

        % Randomly generate viewing frame
        h_view{i,j} = [quat2rotm(rand(1,4)), 2*rand(3,1)-1;
            0, 0, 0, 1];

        % Load random via-point poses
        t_via_1 = trials.t_via{1}(i);
        g_via_1 = trials.g_via{1}(:,:,i);

        t_via_2 = trials.t_via{2}(i);
        g_via_2 = trials.g_via{2}(:,:,i);

        % Initiate class
        res_primp{i,j}.group_name = param.group_name;
        res_primp_no_equi{i,j}.group_name = param.group_name;

        t_start = tic;

        primp_obj = PRIMP(g_mean.matrix, cov_t, param);
        primp_obj.set_new_view_frame(h_view{i,j});

        % Condition on via-point poses
        primp_obj.get_condition_pdf(t_via_1, g_via_1, cov_via);
        primp_obj.get_condition_pdf(t_via_2, g_via_2, cov_via);
        g_samples = primp_obj.get_samples();

        time.primp(i,j) = toc(t_start);

        % Baseline: Not use equivariance property
        primp_no_equi_obj = PRIMP(g_mean.matrix, cov_t, param);

        % Condition on via-point poses
        primp_no_equi_obj.get_condition_pdf(t_via_1, g_via_1, cov_via);
        primp_no_equi_obj.get_condition_pdf(t_via_2, g_via_2, cov_via);
        g_samples_no_equi = primp_no_equi_obj.get_samples();

        % Transform all samples
        n_sample = length(g_samples_no_equi);
        n_step = size(g_samples_no_equi{1}, 3);
        g_samples_no_equi_view = cell(param.n_sample);
        for m = 1:n_sample
            for n = 1:n_step
                g_samples_no_equi_view{m}(:,:,n) = get_conjugation(...
                    g_samples_no_equi{m}(:,:,n), inv(h_view{i,j}), group_name{j});
            end
        end

        time.primp_no_equi(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_primp{i,j} =...
            generate_pose_struct(g_samples, param.group_name);
        res_primp_no_equi{i,j}.original =...
            generate_pose_struct(g_samples_no_equi, param.group_name);
        res_primp_no_equi{i,j}.transformed =...
            generate_pose_struct(g_samples_no_equi_view, param.group_name);

        % Similarity of samples
        d_sim(i,:,j) = evaluate_traj_distribution(res_primp{i,j}, res_primp_no_equi{i,j}.transformed);

        % Distance to desired pose
        g_via_view_2 = get_conjugation(g_via_2, inv(h_view{i,j}), group_name{j});
        d_via.primp(i,:,j) = evaluate_desired_pose(res_primp{i,j}, g_via_view_2, t_via_2);
        d_via.primp_no_equi(i,:,j) = evaluate_desired_pose(res_primp_no_equi{i,j}.transformed, g_via_view_2, t_via_2);
    end

end

%% Evaluation of benchmarks
% Store distance results
res_filename = strcat(result_folder, "result_lfd_primp.mat");
save(res_filename, "time", "d_sim", "d_via", "h_view");

% Display and store command window
diary_filename = strcat(result_folder, "result_lfd_primp.txt");
if exist(diary_filename, 'file') ; delete(diary_filename); end
diary(diary_filename);

for j = 1:length(group_name)
    disp('===============================================================')
    disp(['Group: ', group_name{j}])

    disp('---- Similarity of samples after change of view (rot, tran):')
    disp(num2str( mean(d_sim(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp('>> PRIMP')
    disp(num2str( mean(d_via.primp(:,:,j), 1) ))

    disp(">> PRIMP without equivariance")
    disp(num2str( mean(d_via.primp_no_equi(:,:,j), 1) ))

    disp('---- Elapsed time:')
    disp('>> PRIMP')
    disp(num2str( mean(time.primp(:,j),1) ))

    disp(">> PRIMP without equivariance")
    disp(num2str( mean(time.primp_no_equi(:,j),1) ))
end

diary off
end