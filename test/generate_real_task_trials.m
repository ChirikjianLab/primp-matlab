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
% demo_type = "transporting";
% demo_type = "scooping";
% demo_type = "opening";

demo_mode = "default";
% demo_mode = "sliding";
% demo_mode = "rotating_left";

% Experiment ID
exp_id = "no obstacle/silver_bowl_03_23_1_s";
% exp_id = "white_bowl_03_30_1_s";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Whether to store trials
is_store = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real/trajectory';

% Demo, key pose and result folder
data_folder = strcat("../data/", dataset_name, "/", demo_type, "/", demo_mode, "/");
key_pose_folder = strcat("../data/experiments/", demo_type, "/", exp_id, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");
mkdir(result_folder);

robot = loadrobot("frankaEmikaPanda");
g_init = robot.getTransform(robot.homeConfiguration, 'panda_link8');

% Load demonstrations
param.n_step = n_step;
param.group_name = group_name;
param.data_folder = data_folder;
filenames = dir(strcat(param.data_folder, "*.json"));
n_demo = length(filenames);
g_demo = parse_demo_trajectory(filenames, param);

idx_demo = ceil(n_demo * rand);

disp(strcat("Demo type and mode: ", demo_type, ", ", demo_mode));

% Load sim2real transform
file_functional_pose = dir(strcat(key_pose_folder, "functional_poses.csv"));
if isempty(file_functional_pose)
    g_sim2real = get_sim2real(demo_type);

else
    functional_pose = readmatrix(strcat(file_functional_pose.folder, "/", file_functional_pose.name));
    key_pose = readmatrix(strcat(file_functional_pose.folder, "/", "key_poses.csv"));

    g_functional = [quat2rotm(functional_pose(1,4:end)),...
        functional_pose(1,1:3)'; 0, 0, 0, 1];
    g_key = [quat2rotm(key_pose(1,4:end)), key_pose(1,1:3)'; 0, 0, 0, 1];
    g_sim2real = g_functional \ g_key;
end

pose_sim2real = homo2pose_quat(g_sim2real);

%% Load data and key poses
if strcmp(demo_type, "pouring") || strcmp(demo_type, "scooping")
    disp(strcat("Experiment ID: ", exp_id));

    % Load object poses
    obj_pose = readmatrix(strcat(key_pose_folder, "perception_0/obj_com_pose.txt"));
    g_obj = [quat2rotm(obj_pose(2,:)), obj_pose(1,1:3)'; 0, 0, 0, 1];
    
    % Generate random trials
    pose_obj = zeros(n_trial, 7);
    for j = 1:n_trial
        % Random offset for each trial
        g_tran = [axang2rotm([0, 0, 1, 0.1 * (2*rand-1)]),...
            [0.1 * (2*rand(2,1)-1); 0]; 0, 0, 0, 1];
        
        if strcmp(demo_type, "pouring")
            % >> TASK 1: pouring
            % Tool frame at goal step
            idx_pour = n_step;
            g_tool = g_demo{idx_demo}.matrix(:,:,idx_pour) / g_sim2real;

            % Start pose
            trials.t_via{1}(j) = 0.0;
            trials.g_via{1}(:,:,j) = g_tran * g_demo{idx_demo}.matrix(:,:,1);
            trials.cov_via{1}(:,:,j) = 1e-8 * eye(6);

            % Key pose
            trials.t_via{2}(j) = 1.0;
            trials.g_via{2}(:,:,j) = g_tran * g_demo{idx_demo}.matrix(:,:,idx_pour);
            trials.cov_via{2}(:,:,j) = 1e-8 * eye(6);

        else
            % >> TASK 3: scooping
            % Tool frame at the middle step
            idx_scoop = floor(0.5 * n_step);
            g_tool = g_demo{idx_demo}.matrix(:,:,idx_scoop) / g_sim2real;

            % Start pose
            trials.t_via{1}(j) = 0.0;
            trials.g_via{1}(:,:,j) = g_tran * g_demo{idx_demo}.matrix(:,:,1);
            trials.cov_via{1}(:,:,j) = 1e-8 * eye(6);

            % Key pose
            trials.t_via{2}(j) = 0.5;
            trials.g_via{2}(:,:,j) = g_tran * g_demo{idx_demo}.matrix(:,:,idx_scoop);
            trials.cov_via{2}(:,:,j) = 1e-8 * eye(6);

            % Goal pose
            trials.t_via{3}(j) = 1.0;
            trials.g_via{3}(:,:,j) = trials.g_via{1}(:,:,j);
            trials.cov_via{3}(:,:,j) = 1e-8 * eye(6);
        end

        % Random placement of the object
        g_obj(1:2,4) = g_tool(1:2,4);
        g_obj_trial = g_tran * g_obj;
        pose_obj(j,:) = homo2pose_quat(g_obj_trial);
    end

else
    for j = 1:n_trial
        % Random placement of the start/goal with rotation around global
        % z-axis only
        g_rand_start = [axang2rotm([0, 0, 1, pi*rand]),...
            [0.05 * (2*rand(2,1)-1); 0]; 0, 0, 0, 1];

        if strcmp(demo_type, "transporting")
            % >> TASK 2: transporting
            g_rand_goal = [axang2rotm([0, 0, 1, pi*rand]),...
                [0.05 * (2*rand(2,1)-1); 0]; 0, 0, 0, 1];
        else
            % >> TASK 4 & 5: opening
            g_rand_goal = g_rand_start;
        end

        % Start pose
        trials.t_via{1}(j) = 0.0;
        trials.g_via{1}(:,:,j) = g_rand_start * g_demo{idx_demo}.matrix(:,:,1);
        trials.cov_via{1}(:,:,j) = 1e-8 * eye(6);

        % Goal pose
        trials.t_via{2}(j) = 1.0;
        trials.g_via{2}(:,:,j) = g_rand_goal * g_demo{idx_demo}.matrix(:,:,end);
        trials.cov_via{2}(:,:,j) = 1e-8 * eye(6);
    end

end

disp("Generated key configurations!")

%% Store poses
if is_store
    n_via = length(trials.t_via);

    if ~strcmp(demo_type, "opening")
        writematrix(pose_sim2real, strcat(result_folder, 'sim2real_transform.csv'));

        if ~strcmp(demo_type, "transporting")
            writematrix(pose_obj, strcat(result_folder, 'object_poses.csv'));
        end
    end

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

%% Function for sim2real transform
function g_sim2real = get_sim2real(demo_type)
g_sim2real = eye(4);

if strcmp(demo_type, "pouring")
    % Cup
    g_sim2real(1:3,1:3) = rpy2r([-pi/2, -pi/4, 0]);
    g_sim2real(1:3,4) = [0; -0.103; -0.07];

elseif strcmp(demo_type, "scooping") || strcmp(demo_type, "transporting")
    % Spoon
    g_sim2real(1:3,1:3) = rpy2r([-pi, 0, pi/4]);
    g_sim2real(1:3,4) = [0; -0.1; 0.103];
end
end
