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

% Number of trials for each key pose
n_trial = 50;

% Type of demonstration
demo_type = "pouring";
demo_mode = "default";

% Experiment ID
exp_id = "cup_silver_bowl_03_23_1";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Whether to store trials
is_store = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real';

% Key pose and result folder
key_pose_folder = strcat("../data/experiment_key_pose/", demo_type, "/", demo_mode, "/", exp_id, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/", demo_mode, "/");
mkdir(result_folder);

robot = loadrobot("frankaEmikaPanda");
g_init = robot.getTransform(robot.homeConfiguration, 'panda_link8');

disp(strcat("Demo type and mode: ", demo_type, ", ", demo_mode));
disp(strcat("Experiment ID: ", exp_id));

%% Load data and key poses
% Load functional, key and object poses
functional_poses = load(strcat(key_pose_folder, exp_id, "_functional_poses.csv"));
key_poses = load(strcat(key_pose_folder, exp_id, "_key_poses.csv"));
obj_pose = readmatrix(strcat(key_pose_folder, "perception_0/obj_com_pose.txt"));
n_pose = size(key_poses, 1);

% Object pose in homo transform
g_obj = [quat2rotm(obj_pose(2,:)), obj_pose(1,1:3)'; 0, 0, 0, 1];

pose_sim2real = zeros(n_pose, 7);
pose_obj = zeros(n_pose * n_trial, 7);
for i = 1:n_pose
    % Relative transform between functional pose to key pose
    g_functional = [quat2rotm(functional_poses(i,4:end)),...
        functional_poses(i,1:3)'; 0, 0, 0, 1];
    g_key = [quat2rotm(key_poses(i,4:end)), key_poses(i,1:3)';
        0, 0, 0, 1];
    g_sim2real = g_functional \ g_key;
    pose_sim2real(i,:) = [g_sim2real(1:3,4)', rotm2quat(g_sim2real(1:3,1:3))];

    % Initial pose of tool and robot frame
    g_tool_init = g_init;
    g_tool_init(1:3,1:3) = g_functional(1:3,1:3) * axang2rotm([0, 1, 0, -pi/1.5]) * axang2rotm([0, 0, 1, -pi/2]);
    g_robot_init = g_tool_init * g_sim2real;

    % Generate random trials for each key pose
    for j = 1:n_trial
        idx = (i-1) * n_trial + j;

        % Random placement of the object
        g_tran = [eye(3), [0.01 * (2*rand(2,1)-1); 0]; 0, 0, 0, 1];

        g_obj_trial = g_tran * g_obj;
        pose_obj(idx,:) = [g_obj_trial(1:3,4)', rotm2quat(g_obj_trial(1:3,1:3))];

        % Start pose
        trials.t_via{1}(idx) = 0.0;
        trials.g_via{1}(:,:,idx) = g_tran * g_robot_init;
        trials.cov_via{1}(:,:,idx) = 1e-8 * eye(6);
        
        % Key pose
        trials.t_via{2}(idx) = 1.0;
        trials.g_via{2}(:,:,idx) = g_tran * g_key;
        trials.cov_via{2}(:,:,idx) = 1e-8 * eye(6);
    end
end

disp("Generated key configurations!")

%% Store poses
if is_store
    n_via = length(trials.t_via);

    writematrix(pose_sim2real, strcat(result_folder, 'sim2real_transform.csv'));
    writematrix(pose_obj, strcat(result_folder, 'object_poses.csv'));

    for k = 1:n_via
        trials_via.num_trial = length(trials.t_via{k});
        trials_via.t_via = trials.t_via{k};
        trials_via.g_via = permute(trials.g_via{k}, [3,1,2]);
        trials_via.cov_via = permute(trials.cov_via{k}, [3,1,2]);

        json_data = jsonencode(trials_via);
        fid = fopen( strcat(result_folder, 'trials_random_via_',...
            num2str(k), '.json'), 'w');
        fprintf(fid, '%s', json_data);
        fclose(fid);
    end

    disp("Stored trials!")
end