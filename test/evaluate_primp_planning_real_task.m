% Evaluation script for planning for real-world experiments
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;
add_paths()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Type of demonstration
demo_type = "pouring/default";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the dataset
dataset_name = 'panda_arm/real';

% Result folder
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type, "/");
joint_traj_folder = strcat(result_folder, "/joint_trajectory/");

pose_sim2real = load(strcat(result_folder, "/sim2real_transform.csv"));
g_sim2real = [quat2rotm(pose_sim2real(1,4:end)), pose_sim2real(1,1:3)';
    0, 0, 0, 1];

%% Load joint trajectory and convert to tool trajectory
robot = loadrobot("frankaEmikaPanda", "DataFormat", "row");

filenames = dir(strcat(joint_traj_folder, "*.csv"));
joint_trajectory = load(strcat(joint_traj_folder, filenames(1).name));
n_step = size(joint_trajectory, 1);

n_trial = length(filenames);
pose_tool = nan(n_step, 7, n_trial);

for i = 1:n_trial
    clc;
    disp(strcat(num2str(i), "/", num2str(n_trial)));

    joint_trajectory = load(strcat(joint_traj_folder, filenames(i).name));

    g_link8 = nan(4, 4, n_step);
    g_tool = nan(4, 4, n_step);
    for j = 1:n_step
        g_link8(:,:,j) = robot.getTransform([joint_trajectory(j,:), 0, 0], 'panda_link8');

        % Transform to tool frame
        g_tool(:,:,j) = g_link8(:,:,j) / g_sim2real;
        pose_tool(j,:,i) = homo2pose_quat(g_tool(:,:,j));
    end
end

%% Save trajectories
trajectory_primp.num_trials = n_trial;
trajectory_primp.num_steps = n_step;
trajectory_primp.pose_format = "[x,y,z,qx,qy,qz,qw]";
trajectory_primp.tool_trajectory = permute(pose_tool, [3,1,2]);

json_data = jsonencode(trajectory_primp);
fid = fopen( strcat(result_folder, 'tool_trajectory_primp_stomp.json'), 'w');
fprintf(fid, '%s', json_data);
fclose(fid);

disp("Tool trajectories saved to file!");

%% Plot
figure;

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];
init = robot.homeConfiguration;

for i = 1:n_step
    [configSol{i}, ~] = ik('panda_link8', g_link8(:,:,i), weights, init);
end
robot.show(configSol{1}, 'Frames', 'off');
hold on; axis equal; axis off;
robot.show(configSol{end}, 'Frames', 'off');

% Tool frames after planning
for j = 1:2:n_step
    trplot(g_tool(:,:,j), 'rgb', 'notext', 'length', 0.05)
end