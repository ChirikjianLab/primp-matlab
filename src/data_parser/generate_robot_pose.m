function [pose, robot, ee_name] = generate_robot_pose(n_state, robot_name)
% generate_robot_pose Generate poses of the distal end of each link
% for a serial robot manipulator
%
% Input
%   n_state   : Number of sampled states of each joint, default = 20
%   robot_name: Name of the robot, default: panda_arm
%
% Output
%   pose      : Poses of the distal end of each link, a structure including
%               Cartesian and exponential coordinates
%   robot     : Robot model in Robotics Toolbox
%   ee_name   : Name of end effector
%
% Author
%   Sipu Ruan, 2023
%
% Dependency
%   Robotics System Toolbox

% Robot parameters
if nargin < 1
    n_state = 20;
end

% Load robot model
[robot_model_name, ee_name, num_link] = load_robot_model(robot_name);
robot = loadrobot(robot_model_name);

% Discrete poses of each link distal end
pose.cartesian = cell(1, num_link);
pose.exponential = cell(1, num_link);
config = homeConfiguration(robot);

for i = 1:num_link
    if i == 1
        body_target = robot.BaseName;
    else
        body_target = robot.Bodies{i-1}.Name;
    end

    body_source = robot.Bodies{i}.Name;

    joint_lim = robot.Bodies{i}.Joint.PositionLimits;
    for k = 1:length(joint_lim)
        if joint_lim(k) == -inf
            joint_lim(k) = -pi;
        elseif joint_lim(k) == inf
            joint_lim(k) = pi;
        end
    end

    for j = 1:n_state
        config(i).JointPosition = (1-(j-1)/(n_state-1)) * joint_lim(1) +...
            (j-1)/(n_state-1) * joint_lim(2);
        g = getTransform(robot, config, body_source, body_target);

        % Cartesian coordinates
        pose.cartesian{i}(:,j) = [g(1:3,4); rotm2quat(g(1:3,1:3))'];

        % exponential coordinates
        pose.exponential{i}(:,j) = get_exp_coord(g);
    end
end