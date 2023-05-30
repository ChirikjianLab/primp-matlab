function [mu, sigma, flag] = get_mean_cov(g, group_name, is_plot)
% get_mean_cov Compute mean and covariance given a set of group elements,
% supports SE(3), PCG(3)
%
% Input
%   g: (d+1) x (d+1) x N homogeneous transformation matrices
%
% Output
%   mu: mean in homogeneous tranformation matrix form
%   sigma: covariance
%
% Author
%   Qianli Ma, 2018
%   Sipu Ruan, 2022
%
% See also
%   get_exp_mapping, get_exp_coord, pose_diff_pcg

if nargin < 3
    is_plot = false;
end

num = size(g,3);
dim = size(g,1);
cov_dim = dim*(dim-1)/2;
sigma = zeros(cov_dim, cov_dim);

%% Mean
% Intialization
mu_log = zeros(6,1);
for i = 1:num
    mu_log = mu_log + get_exp_coord(g(:,:,i), group_name);
end
mu = get_exp_mapping(1/num * mu_log, group_name);

% Iterative process to calculate the true Mean
mu_log = ones(dim*(dim-1)/2, 1);
max_num = 10;
tol = 1e-5;
count = 1;
while norm(mu_log, 'fro') >= tol && count <= max_num
    mu_log = zeros(dim*(dim-1)/2, 1);
    for i = 1:num
        d_g = get_rel_pose(mu, g(:,:,i), group_name);
        d_g_log = get_exp_coord(d_g, group_name);
        mu_log = mu_log + d_g_log;
    end
    mu = mu * get_exp_mapping(1/num * mu_log, group_name);
    count = count+1;
end

if count > max_num
    warning(['Cannot obtain correct mean pose, error: ',...
        num2str( norm(mu_log, 'fro') )]);
    flag = false;
    return;
end

%% Covariance
for i = 1:num
    d_g = get_rel_pose(mu, g(:,:,i), group_name);
    y_i = get_exp_coord(d_g, group_name);
    sigma = sigma + y_i*y_i';
end
sigma = 1/num * sigma;

%% Plot frames and mean
if is_plot
    len = 0.1;
    figure; hold on; axis equal;
    trplot(mu, 'color', 'r', 'length', len)
    for i = 1:num
        trplot(g(:,:,i), 'color', 'b', 'length', len)
    end
end

flag = true;

end