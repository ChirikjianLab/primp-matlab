function [g_mean, cov_t] = get_pdf_from_demo(g_demo, group_name)
% get_pdf_from_demo Compute probability distribution from demonstrations
%
% Input
%   g_demo    : Demonstration set
%   group_name: Name of the group
% 
% Output
%   g_mean    : Mean of the demonstrations
%   cov_t     : Covariance matrix at each time step
%
% Author
%   Sipu Ruan, 2022

n_demo = length(g_demo);
n_step = size(g_demo{1}.matrix, 3);

% Extract frames for each demo, each step is a set of SE(3)
g_step = cell(n_step, 1);
for i = 1:n_demo
    for j = 1:n_step
        g_step{j}(:,:,i) = g_demo{i}.matrix(:,:,j);
    end
end

% Relative poses between adjacent frames
g_demo_rel = cell(n_demo, 1);
g_step_rel = cell(n_step, 1);
for i = 1:n_demo
    for j = 1:n_step-1
        g_rel = get_rel_pose(g_demo{i}.matrix(:,:,j),...
            g_demo{i}.matrix(:,:,j+1), group_name);

        g_demo_rel{i}(:,:,j) = g_rel;
        g_step_rel{j}(:,:,i) = g_rel;
    end
end

% Compute mean using original trajectories
g_mean_matrix = cell(1);
for i = 1:n_step
    [g, ~, flag] = get_mean_cov(g_step{i}, group_name);

    if flag
        g_mean_matrix{1} = cat(3, g_mean_matrix{1}, g);
    end
end

% Apply GORA for optimal mean trajectory
gora_obj = gora(g_mean_matrix{1}, n_step);
gora_obj.run();
g_mean_matrix{1} = gora_obj.g_opt;

g_mean = generate_pose_struct(g_mean_matrix, group_name);
g_mean = g_mean{1};

% Compute covariance using relative frames
cov_t = nan(6,6,n_step);
[~, cov_t(:,:,1)] = get_mean_cov(g_step{1}, group_name);
for i = 1:n_step
    if i ~= n_step
        [~, cov_t(:,:,i+1)] = get_mean_cov(g_step_rel{i}, group_name);
    end

    % Add small variances to avoid singularity
    cov_t(:,:,i) = cov_t(:,:,i) + 1e-6 * eye(6);
end
end