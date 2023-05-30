% Script for a robot following the demonstration trajectories
%
%  Author
%    Sipu Ruan, 2022

close all; clear; clc;

% Dataset and demo set info
dataset_name = 'panda_arm';
% dataset_name = 'lasa_handwriting/pose_data';

demo_type = load_dataset_param(dataset_name);
data_folder = strcat("../data/", dataset_name, "/", demo_type, "/");

% Load robot
is_load_robot = true;

if is_load_robot
    robot = loadrobot('frankaEmikaPanda');
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = ones(1,6);
    initialguess = robot.homeConfiguration;
end

% Demo trajectories
for i = 1:length(demo_type)
    argin.n_step = 50;
    argin.group_name = "PCG";
    argin.data_folder = data_folder(i);

    % Load demos
    filenames = dir(strcat(argin.data_folder, "*.json"));
    g_demo = parse_demo_trajectory(filenames, argin);

    % Compute trajectory distribution from demonstrations
    [g_mean, cov_t] = get_pdf_from_demo(g_demo, argin.group_name);
    n_demo = length(g_demo);

    %% PLOT: Demonstrations
    figure;

    % (Optional) Robot following mean trajectory
    if is_load_robot
        for j = 1:20:argin.n_step
            config = ik('panda_link8', g_mean.matrix(:,:,j), weights,...
                initialguess);
            show(robot, config);
            hold on; axis off;
        end
    end

    hold on; axis equal; axis off;

    % Demos
    for j = 1:n_demo
        plot3(g_demo{j}.pose(1,:), g_demo{j}.pose(2,:),...
            g_demo{j}.pose(3,:))
    end

    % Prior mean poses
    pose_mean = g_mean.pose;
    plot3(pose_mean(1,:), pose_mean(2,:), pose_mean(3,:),...
        'b-', 'LineWidth', 3)
    plot3(pose_mean(1,1), pose_mean(2,1), pose_mean(3,1), 'go',...
        'LineWidth', 1.5)
    plot3(pose_mean(1,end), pose_mean(2,end), pose_mean(3,end), 'r*',...
        'LineWidth', 1.5)

    % Save figure
    fig_name = strrep(demo_type{i}, '/', '_');
    saveas(gcf, [fig_name, '.png'])
end