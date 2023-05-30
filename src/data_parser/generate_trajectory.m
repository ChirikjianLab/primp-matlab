function [g_mean, cov_t] = generate_trajectory(argin)
% Generate initial trajectory
%
%  Input
%    argin : parameters for randomly generated trajectory
%
%  Output
%    g_mean: Structure that stored mean trajectory
%              matrix -- 4 x 4 x n_step matrix storing mean trajectory
%              pose   -- pose coordinate: cartesian, exponential
%    cov_t : 6 x 6 x n_step matrix storing covariance of each pose
%
%  Author
%    Sipu Ruan, 2022
%
%  See also
%    generate_pose_struct, get_exp_coord, Robotics System Toolbox

% Simulate robot workspace trajectory
argin.robot.DataFormat = 'column';
body_source = argin.robot.BodyNames{9};

q_0 = argin.robot.randomConfiguration();
q_n = argin.robot.randomConfiguration();
n_joint = length(q_0);

t_span = 0:1/(argin.n_step-1):1;

% Mean configurations and end effector poses
q_mean = nan(n_joint, argin.n_step);
for i = 1:n_joint
    q_mean(i,:) = interp1([0,1], [q_0(i), q_n(i)], t_span);
end

g = cell(1);
for j = 1:argin.n_step
    g{1}(:,:,j) = argin.robot.getTransform(q_mean(:,j), body_source);
end

g_mean = generate_pose_struct(g, argin.group_name);
g_mean = g_mean{1};

% Initial covariance of the trajectory
cov_t = nan(argin.cov_dim, argin.cov_dim, argin.n_step);
cov_init = eye(argin.cov_dim);
cov_init(1:3,1:3) = 5*eye(3);

for i = 1:argin.n_step
    cov_t(:,:,i) = argin.cov_init_scale *...
        norm(1/argin.n_step * get_exp_coord(g_mean.matrix(:,:,1)\...
        g_mean.matrix(:,:,end))) * cov_init;
end