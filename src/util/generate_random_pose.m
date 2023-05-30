function [g_via, cov_via, step_via] = generate_random_pose(g_demo,...
    t_via, scale)
% generate_random_pose Generate random pose
%
% Input
%   g_demo  : Set of demonstrations
%   t_via   : Time step of the via pose
%   scale   : Scale of the mean and covariance for the via pose
%
% Output
%   g_via   : Generated random via pose
%   cov_via : Generated random covariance of the via pose
%   step_via: Index of the time step
%
% Author
%   Sipu Ruan, 2022

n_step = size(g_demo.matrix, 3);

% Via pose deviation from initial, in exponential coordinates
via_pose_deviation_exp_coord = [pi*1e-2*rand(3,1); 0.1*rand(3,1)];

% Desired via pose and covariance
step_via = floor(t_via*n_step);
if step_via <= 0
    step_via = 1;
elseif step_via >= n_step
    step_via = n_step;
end

g_via = g_demo.matrix(:,:,step_via) *...
    expm_SE(scale.mean * via_pose_deviation_exp_coord);
cov_via = scale.covariance * rand() * diag([4 4 4 1 1 1]);
end