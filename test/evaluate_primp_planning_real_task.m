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
% demo_type = "pouring/default";
% demo_type = "transporting/default";
% demo_type = "scooping/default";
demo_type = "opening/sliding";
% demo_type = "opening/rotating_left";

% Type of planning scene
scene_type = "empty";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Result folder
primp_result_folder = strcat("../result/benchmark/panda_arm/real/", demo_type);
result_folder = strcat("../result/benchmark/planning/", scene_type, "/", demo_type, "/");

%% Load joint trajectory and convert to tool trajectory
robot = loadrobot("frankaEmikaPanda", "DataFormat", "row");

% Load sim2real transform
try
    pose_sim2real = load(strcat(primp_result_folder, "/sim2real_transform.csv"));
    g_sim2real = [quat2rotm(pose_sim2real(1,4:end)), pose_sim2real(1,1:3)';
        0, 0, 0, 1];
catch
    g_sim2real = robot.getTransform(robot.homeConfiguration, "panda_link8", "panda_leftfinger");
end

% Load reference trajectory mean and transform to tool frame
filenames = dir(strcat(primp_result_folder, "/primp_PCG/*.csv"));
n_ref = length(filenames);
pose_ref = cell(n_ref, 1);
g_ref = cell(n_ref, 1);
for i = 1:n_ref
    pose_ref{i} = load(strcat(filenames(i).folder, "/", filenames(i).name));
    
    for j = 1:size(pose_ref{i},1)
        g_ref{i}(:,:,j) = [axang2rotm(pose_ref{i}(j,4:end)), pose_ref{i}(j,1:3)';
            0, 0, 0, 1] / g_sim2real;
    end
end

filenames = dir(strcat(result_folder, "*.json"));
n_experiment = length(filenames);
for k = 1:n_experiment
    % Load .json file for each experiment
    json_data_in = jsondecode( fileread(strcat(result_folder, filenames(k).name)) );
    n_trial = json_data_in.num_trials;
    idx_failed = [];
    flag_task = nan(1,n_trial);

    for i = 1:n_trial
        clc;
        disp(strcat("Benchmark: ", num2str(k), "/", num2str(n_experiment)));
        disp(strcat("Trial: ", num2str(i), "/", num2str(n_trial)));

        joint_trajectory = json_data_in.joint_trajectory{i};
        n_step = length(joint_trajectory);
        if n_step > 50
            n_step = 50;
        end

        g_link8 = nan(4, 4, n_step);
        g_tool = nan(4, 4, n_step);

        % Convert to SE(3) transformation matrix
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

        % Evaluate from tool trajectory
        flag_task(i) = is_task_success(g_tool, g_ref{i}, demo_type);
        disp("Direct evaluation without simulation done!")
    end

    %% Save to files
    % Save tool trajectories
    tool_traj_data.num_trials = n_trial;
    tool_traj_data.num_steps = n_step;
    tool_traj_data.cost_name = json_data_in.cost_name;
    tool_traj_data.task_name = json_data_in.task_name;
    tool_traj_data.pose_format = "[x,y,z,qx,qy,qz,qw]";
    tool_traj_data.idx_failed = idx_failed;
    tool_traj_data.tool_trajectory = permute(pose_tool, [3,1,2]);
    
    json_data_out = jsonencode(tool_traj_data, 'PrettyPrint', true);
    fid = fopen( strcat(result_folder, 'tool_trajectory_',...
        json_data_in.cost_name, '.json'), 'w');
    fprintf(fid, '%s', json_data_out);
    fclose(fid);

    
    disp("Tool trajectories saved to file!");

    % Save direct evaluation results
    if ~isnan(flag_task(1))
        task_success_data.flag_task = flag_task;
        task_success_data.task_success_rate = sum(flag_task)/n_trial;

        json_data_out = jsonencode(task_success_data, 'PrettyPrint', true);
        fid = fopen( strcat(result_folder, 'task_success_rate_',...
            json_data_in.cost_name, '.json'), 'w');
        fprintf(fid, '%s', json_data_out);
        fclose(fid);

        disp(strcat("Task success rate: ", num2str(task_success_data.task_success_rate)));
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
end

%% Function for direct evaluation
function flag = is_task_success(g_tool, g_ref, demo_type)
flag = nan;
n_step = size(g_tool, 3);

if strcmp(demo_type, "transporting/default")
        % Orientation align with global z-axis
        axis_ref = [0; 0; -1];
        dist = nan(1,n_step);

        for i = 1:n_step
            axis_tool = g_tool(1:3,3,i);
            dist(i) = acos(dot(axis_tool, axis_ref));
        end

        flag = all(abs(dist) <= 20/180*pi);

elseif strcmp(demo_type, "opening/sliding") || strcmp(demo_type, "opening/rotating_left")
%         figure; hold on; axis equal;

        % Position relative to starting point
        for i = 1:n_step
            g_rel_ref = g_ref(:,:,1) \ g_ref(:,:,i);
            g_rel_tool = g_tool(:,:,1) \ g_tool(:,:,i);

            g_diff = get_rel_pose(g_rel_ref, g_rel_tool, 'PCG');

            dist(1,i) = norm(logm_SO(g_diff(1:3,1:3)));
            dist(2,i) = norm(g_diff(1:3,4));

%             plot3(g_rel_ref(1,4), g_rel_ref(2,4), g_rel_ref(3,4), 'k*')
%             plot3(g_rel_tool(1,4), g_rel_tool(2,4), g_rel_tool(3,4), 'b.')
        end

         flag = all(dist(1,:) <= 20/180*pi) && all(dist(2,:) <= 0.1);
end
end