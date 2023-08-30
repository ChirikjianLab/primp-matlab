function [pose, robot, ee_name] = generate_robot_ee_pose(n_state, robot_name)
% generate_robot_ee_pose Generate poses of the end effector for a serial
% robot manipulator
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
    n_state = 500;
end

% Load robot model
[robot_model_name, ee_name] = load_robot_model(robot_name);
robot = loadrobot(robot_model_name);

% Discrete poses of end effector
for j = 1:n_state
    config = randomConfiguration(robot);
    g = getTransform(robot, config, ee_name, robot.BaseName);

    % Cartesian coordinates
    pose.cartesian{1}(:,j) = [g(1:3,4); rotm2quat(g(1:3,1:3))'];

    % exponential coordinates
    pose.exponential{1}(:,j) = get_exp_coord(g);
end
