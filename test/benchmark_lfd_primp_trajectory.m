% Benchmark script for PRIMP and store learned trajectory distribution
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()

% Name of the dataset
dataset_name = 'panda_arm';

demo_type = load_dataset_param(dataset_name);

%% Run benchmark for each demo type
id = [5, 6, 7, 8, 10];
for i = 1:length(id)
    run_benchmark(dataset_name, demo_type{id(i)});
end

function run_benchmark(dataset_name, demo_type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of samples on distribution
n_sample = 5;

% Number of time steps
n_step = 50;

% Group name
group_name = 'PCG';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/",...
    demo_type, "/");
result_prefix = strcat(result_folder, 'primp_', group_name, '/');

mkdir(result_folder);
mkdir(result_prefix);

%% Load and parse demo data
argin.n_step = n_step;
argin.data_folder = data_folder;
argin.group_name = 'SE';

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Load random via/goal poses
trials = load_random_trials(result_folder);
n_trial = length(trials.t_via{1});

disp("Loaded randomly generated configurations!");

%% Benchmark
% Compute trajectory distribution from demonstrations
[g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name);

for i = 1:n_trial
    clc;
    disp('Benchmark: PRIMP (ours)')
    disp(['Dataset: ', dataset_name])
    disp(['Demo type: ', demo_type])
    disp([num2str(i/(n_trial) * 100), '%'])

    % Initiate class
    param.n_sample = n_sample;
    param.group_name = group_name;
    primp_obj = PRIMP(g_mean.matrix, cov_t, param);

    % Condition on via points
    for j = 1:length(trials.t_via)
        [mean_cond, cov_cond] = primp_obj.get_condition_pdf(...
            trials.t_via{j}(i), trials.g_via{j}(:,:,i),...
            trials.cov_via{j}(:,:,i));
    end
    g_samples = primp_obj.get_samples();

    % Convert to pose
    pose_samples = cell(n_sample, 1);
    for m = 1:n_sample
        n_step = length(g_samples{m});
        pose_samples{m} = nan(n_step, 7);
        for n = 1:n_step
            pose_samples{m}(n,:) = homo2pose_quat(g_samples{m}(:,:,n));
        end
    end

    pose_cond_mean = nan(n_step, 7);
    for m = 1:n_step
        pose_cond_mean(m,:) = homo2pose_axang(mean_cond(:,:,m));
    end

    cov_cond_step = zeros(6, 6, n_step);
    for m = 1:n_step
        idx_block = 6*(m-1)+1:6*m;
        cov_cond_step(:,:,m) = cov_cond(idx_block, idx_block);
    end

    % Store learned trajectory
    % Mean and covariance
    prob_data.num_steps = n_step;
    prob_data.mean = permute(mean_cond, [3,1,2]);
    prob_data.covariance_joint = cov_cond;
    prob_data.covariance_step = permute(cov_cond_step, [3,1,2]);

    json_data = jsonencode(prob_data);
    fid = fopen(strcat(result_prefix, 'reference_density_', num2str(i),...
        '.json'), 'w');
    fprintf(fid, '%s', json_data);
    fclose(fid);

    writematrix(pose_cond_mean, strcat(result_prefix,...
        'reference_density_mean_', num2str(i),'.csv'));

    % Samples
    sample_data.num_samples = n_sample;
    sample_data.num_steps = n_step;
    sample_data.samples = pose_samples;

    json_data = jsonencode(sample_data);
    fid = fopen(strcat(result_prefix, 'samples_', num2str(i),'.json'), 'w');
    fprintf(fid, '%s', json_data);
    fclose(fid);
end

end