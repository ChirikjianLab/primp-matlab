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
result_folder = strcat("../result/benchmark/", dataset_name, "/", demo_type);
joint_traj_folder = strcat(result_folder, "/joint_trajectory/");
tool_traj_folder = strcat(result_folder, "/tool_trajectory/");
mkdir(tool_traj_folder);

pose_sim2real = load(strcat(result_folder, "/sim2real_transform.csv"));
g_sim2real = [quat2rotm(pose_sim2real(1,4:end)), pose_sim2real(1,1:3)';
    0, 0, 0, 1];

%% Load joint trajectory and convert to tool trajectory
robot = loadrobot("frankaEmikaPanda", "DataFormat", "row");

filenames = dir(strcat(joint_traj_folder, "*.json"));
n_experiment = length(filenames);

for k = 1:n_experiment
    % Load .json file for each experiment
    json_data_in = jsondecode( fileread(strcat(joint_traj_folder, filenames(k).name)) );
    n_trial = json_data_in.num_trials;
    idx_failed = [];
    
    for i = 1:n_trial
        clc;
        disp(strcat("Benchmark: ", num2str(k), "/", num2str(n_experiment)));
        disp(strcat("Trial: ", num2str(i), "/", num2str(n_trial)));
        
        joint_trajectory = json_data_in.joint_trajectory{i};
        n_step = size(joint_trajectory, 1);
        g_link8 = nan(4, 4, n_step);
        g_tool = nan(4, 4, n_step);

        for j = 1:n_step
            joint_config = str2double(joint_trajectory{j})';

            try
                g_link8(:,:,j) = robot.getTransform([joint_config, 0, 0], 'panda_link8');
                
                % Transform to tool frame
                g_tool(:,:,j) = g_link8(:,:,j) / g_sim2real;
                pose_tool(j,:,i) = homo2pose_quat(g_tool(:,:,j));
            catch
                idx_failed = [idx_failed, i];
                pose_tool(j,:,i) = nan(1,7);
            end
        end
    end

    %% Save tool trajectories
    tool_traj_data.num_trials = n_trial;
    tool_traj_data.num_steps = n_step;
    tool_traj_data.cost_name = json_data_in.cost_name;
    tool_traj_data.task_name = json_data_in.task_name;
    tool_traj_data.pose_format = "[x,y,z,qx,qy,qz,qw]";
    tool_traj_data.idx_failed = idx_failed;
    tool_traj_data.tool_trajectory = permute(pose_tool, [3,1,2]);

    json_data_out = jsonencode(tool_traj_data, 'PrettyPrint', true);
    fid = fopen( strcat(tool_traj_folder, 'tool_trajectory_',...
        json_data_in.cost_name, '.json'), 'w');
    fprintf(fid, '%s', json_data_out);
    fclose(fid);

    disp("Tool trajectories saved to file!");
end

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