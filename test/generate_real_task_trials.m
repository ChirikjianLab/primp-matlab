% Script for loading key poses and generating trials for real tasks
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of steps for interpolated trajectories
n_step = 50;

% Type of demonstration
demo_type = "pouring/default";

% Experiment ID
exp_id = "cup_silver_bowl_03_23_2";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Scaling of via pose mean and covariance
VIA_POSE_SCALE.mean = [1e-3 * ones(3,1); 1e-4 * ones(3,1)];
VIA_POSE_SCALE.covariance = 1e-6;

% Whether to store trials
is_store = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real';

% Data and result folder
key_pose_folder = strcat("../data/experiment_key_pose/", demo_type, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");
mkdir(result_folder);

%% Load data and key poses
functional_poses = load(strcat(key_pose_folder, exp_id, "_functional_poses.csv"));
key_poses = load(strcat(key_pose_folder, exp_id, "_key_poses.csv"));
n_pose = size(key_poses, 1);

pose_sim2real = zeros(n_pose,7);
for i = 1:n_pose
    % Relative transform between functional pose to key pose
    g_functional = [quat2rotm(functional_poses(i,4:end)),...
        functional_poses(i,1:3)'; 0, 0, 0, 1];
    g_key = [quat2rotm(key_poses(i,4:end)), key_poses(i,1:3)'; 
        0, 0, 0, 1];
    g_sim2real = g_functional \ g_key;
    pose_sim2real(i,:) = [g_sim2real(1:3,4)', rotm2quat(g_sim2real(1:3,1:3))];

    % Via-point pose
    trials.t_via{1}(i) = 0.0;
    trials.step_via{1}(i) = 1;
    trials.g_via{1}(:,:,i) = g_key;
    trials.cov_via{1}(:,:,i) = 1e-5 * eye(6);

    trials.t_via{2}(i) = 1.0;
    trials.step_via{2}(i) = n_step;
    trials.g_via{2}(:,:,i) = g_key;
    trials.cov_via{2}(:,:,i) = 1e-5 * eye(6);

    % Store poses
    if is_store
        n_via = length(trials.t_via);

        writematrix(pose_sim2real, strcat(result_folder, 'sim2real_transform.csv'));

        for j = 1:n_via
            trials_via.t_via = trials.t_via{j};
            trials_via.g_via = permute(trials.g_via{j}, [3,1,2]);
            trials_via.cov_via = permute(trials.cov_via{j}, [3,1,2]);

            json_data = jsonencode(trials_via);
            fid = fopen( strcat(result_folder, 'trials_random_via_',...
                num2str(j), '.json'), 'w');
            fprintf(fid, '%s', json_data);
            fclose(fid);
        end
    end
end

disp("Generated key configurations!")

