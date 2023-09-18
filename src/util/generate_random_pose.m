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

% Via point deviation from initial, in exponential coordinates
via_pose_deviation = [pi*1e-2*rand(3,1); 2*rand(3,1)-1];

% Desired via pose and covariance
step_via = floor(t_via*n_step);
if step_via <= 0
    step_via = 1;
elseif step_via >= n_step
    step_via = n_step;
end

% Deviate from mean
g_via = eye(4);
g_via(1:3,1:3) = g_demo.matrix(1:3,1:3,step_via) *...
    expm_SO(scale.mean(1:3) .* via_pose_deviation(1:3));
g_via(1:3,4) = g_demo.matrix(1:3,4,step_via) +...
    scale.mean(4:6) .* via_pose_deviation(4:6);

% Covariance of via point uncertainty
cov_via = scale.covariance * rand() * diag([4 4 4 1 1 1]);
end