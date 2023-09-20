% Script for generating trials (random goal and via points) for benchmark
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;

dataset_name = {'panda_arm', 'lasa_handwriting/pose_data'};
n_trial = 50;

scale.mean = zeros(6,1);
scale.covariance = 1e-4;

% Factor for extrapolation
lambda_ex = 0;
% lambda_ex = 10;

for j = 1:length(dataset_name)
    demo_types = load_dataset_param(dataset_name{j});

    % Via point deviation based on dataset name
    switch dataset_name{j}
        case 'panda_arm'
            scale.mean(1:3) = 1e-4 * ones(3,1) * lambda_ex;
            scale.mean(4:6) = 1e-3 * rand(3,1) + lambda_ex;

        case 'lasa_handwriting/pose_data'
            scale.mean(4:6) = [1e-3 * rand(2,1) + lambda_ex; 0];
    end

    for i = 1:length(demo_types)
        generate_trials(dataset_name{j}, demo_types{i}, n_trial, scale, false);
    end
end

%% Function for generating benchmark trials
function generate_trials(dataset_name, demo_type, n_trial, scale, isplot)
clc;
disp(['Dataset: ', dataset_name])
disp(['Demo type: ', demo_type])

data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");
mkdir(result_folder);

% Load and parse demo data
argin.n_step = 50;
argin.data_folder = data_folder;
argin.group_name = 'SE';

filenames = dir(strcat(argin.data_folder, "*.json"));
g_demo = parse_demo_trajectory(filenames, argin);

% Generate random goal/via points
id_demo = ceil(rand*length(g_demo));
trials = generate_random_trials(g_demo{id_demo}, n_trial, scale, result_folder);

if isplot
    % Plot trials
    figure; hold on; axis equal;

    for i = 1:length(trials.g_via)
        g_via = trials.g_via{i};

        for j = 1:size(g_via, 3)
            plot3(g_via(1,4,j), g_via(2,4,j), g_via(3,4,j), 'b*');
        end
    end
end

end