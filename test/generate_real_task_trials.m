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
% demo_type = "pouring";
demo_type = "transporting";
% demo_type = "scooping";
% demo_type = "opening";

demo_mode = "default";
% demo_mode = "sliding";
% demo_mode = "rotating_left";

% Experiment ID
% exp_id = "cup_silver_bowl_03_23_1";
exp_id = "spoon_white_bowl_03_30_1";

% Group name: 'SE', 'PCG'
group_name = 'PCG';

% Whether to store trials
is_store = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real';

% Demo, key pose and result folder
data_folder = strcat("../data/", dataset_name, "/", demo_type, "/", demo_mode, "/");
key_pose_folder = strcat("../data/experiment_key_pose/", demo_type, "/", demo_mode, "/", exp_id, "/");
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/", demo_mode, "/");
mkdir(result_folder);

robot = loadrobot("frankaEmikaPanda");
g_init = robot.getTransform(robot.homeConfiguration, 'panda_link8');

% Load demonstrations
filenames = dir(strcat(data_folder, "*.json"));
file = jsondecode( fileread(strcat(data_folder, filenames(1).name)) );
g_demo = permute(file.trajectory, [2,3,1]);

disp(strcat("Demo type and mode: ", demo_type, ", ", demo_mode));

%% Load data and key poses
if strcmp(demo_type, "pouring") || strcmp(demo_type, "scooping")
    disp(strcat("Experiment ID: ", exp_id));

    % Load functional, key and object poses
    functional_poses = load(strcat(key_pose_folder, "functional_poses.csv"));
    key_poses = load(strcat(key_pose_folder, "key_poses.csv"));
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
        pose_sim2real(i,:) = homo2pose_quat(g_sim2real);

        % Generate random trials for each key pose
        for j = 1:n_trial
            idx = (i-1) * n_trial + j;

            % Random placement of the object
            g_tran = [eye(3), [0.01 * (2*rand(2,1)-1); 0]; 0, 0, 0, 1];

            g_obj_trial = g_tran * g_obj;
            pose_obj(idx,:) = homo2pose_quat(g_obj_trial);

            if strcmp(demo_type, "pouring")
                % >> TASK 1: pouring
                % Initial pose of tool and robot frame
                g_tool_init = g_init;
                g_tool_init(1:3,1:3) = g_functional(1:3,1:3) *...
                    axang2rotm([0, 1, 0, -pi/1.5]) * axang2rotm([0, 0, 1, -pi/2]);
                g_robot_init = g_tool_init * g_sim2real;

                % Start pose
                trials.t_via{1}(idx) = 0.0;
                trials.g_via{1}(:,:,idx) = g_tran * g_robot_init;
                trials.cov_via{1}(:,:,idx) = 1e-8 * eye(6);

                % Key pose
                trials.t_via{2}(idx) = 1.0;
                trials.g_via{2}(:,:,idx) = g_tran * g_key;
                trials.cov_via{2}(:,:,idx) = 1e-8 * eye(6);

            else
                % >> TASK 3: scooping
                % Initial pose of tool and robot frame
                g_robot_init = g_init;
                g_robot_init(1:3,1:3) = g_demo(1:3,1:3);
                g_robot_init(1:3,4) = g_robot_init(1:3,4) - [0;0;0.3];

                % Start pose
                trials.t_via{1}(idx) = 0.0;
                trials.g_via{1}(:,:,idx) = g_tran * g_robot_init;
                trials.cov_via{1}(:,:,idx) = 1e-8 * eye(6);

                % Key pose
                trials.t_via{2}(idx) = 0.5;
                trials.g_via{2}(:,:,idx) = g_tran * g_key;
                trials.cov_via{2}(:,:,idx) = 1e-8 * eye(6);

                % Goal pose
                trials.t_via{3}(idx) = 1.0;
                trials.g_via{3}(:,:,idx) = trials.g_via{1}(:,:,idx);
                trials.cov_via{3}(:,:,idx) = 1e-8 * eye(6);
            end
        end
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
        trials.g_via{1}(:,:,j) = g_rand_start * g_demo(:,:,1);
        trials.cov_via{1}(:,:,j) = 1e-8 * eye(6);

        % Goal pose
        trials.t_via{2}(j) = 1.0;
        trials.g_via{2}(:,:,j) = g_rand_goal * g_demo(:,:,end);
        trials.cov_via{2}(:,:,j) = 1e-8 * eye(6);
    end

end

disp("Generated key configurations!")

%% Store poses
if is_store
    n_via = length(trials.t_via);

    if strcmp(demo_type, "pouring") || strcmp(demo_type, "scooping")
        writematrix(pose_sim2real, strcat(result_folder, 'sim2real_transform.csv'));
        writematrix(pose_obj, strcat(result_folder, 'object_poses.csv'));
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
