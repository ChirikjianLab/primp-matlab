function dist = compute_mahalanobis_distance(g, mu, cov, group_name)
% compute_mahalanobis_distance Compute Mahalanobis distance measure from
% for Lie group
%
% Input
%   g   : Queried element
%   mu  : Mean
%   cov : Covariance
%
% Output
%   dist: Computed mahalanobis distance measure
%
% Author
%   Sipu Ruan, 2023
%
% Dependency
%   get_rel_pose, get_exp_coord

n_step = size(g, 3);

dist = nan(n_step, 1);
for i = 1:n_step
    g_diff = get_rel_pose(mu, g(:,:,i), group_name);
    dist(i) = get_exp_coord(g_diff, group_name)' / cov *...
        get_exp_coord(g_diff, group_name);
end