function d_mean_traj = evaluate_traj_distribution(traj_res, traj_init)
% evaluate_traj_distribution Evaluate similarity between two trajectory
% distributions. Distances for rotation and translation parts are
% separated. 
% 
% Using DTW for signal similarity. Before using, 
% compile "/src/util/dtw_c.c": mex dtw_c.c
%  Reference: Quan Wang (2023). Dynamic Time Warping (DTW) 
% (https://www.mathworks.com/matlabcentral/fileexchange/43156-dynamic-time-warping-dtw), MATLAB Central File Exchange.
%
% Input
%   traj_res   : Resulting trajectory samples
%   traj_init  : Initial set of trajectories (mean of defined distribution
%   or demonstrated trajectories)
%
% Output
%   d_mean_traj: Distance metric, [rot, tran]
%
% Author
%   Sipu Ruan, 2022

n_demo = length(traj_init);
n_sample = length(traj_res);
n_step = size(traj_init{1}.matrix, 3);

% Distance to initial trajectory mean ([rot, tran])
d_mean_traj = [0, 0];
for k = 1:n_demo
    for j = 1:n_sample
        rot_init = traj_init{k}.matrix(1:3,1:3,:);
        tran_init = traj_init{k}.matrix(1:3,4,:);
        
        rot_res = traj_res{j}.matrix(1:3,1:3,:);
        tran_res = traj_res{j}.matrix(1:3,4,:);

        % Reshape to 2D matrix to (n_step, matrix elements)
        rot_init = reshape(rot_init,...
            [size(rot_init, 1)*size(rot_init, 2), size(rot_init, 3)])';
        tran_init = reshape(tran_init,...
            [size(tran_init, 1)*size(tran_init, 2), size(tran_init, 3)])';
        rot_res = reshape(rot_res,...
            [size(rot_res, 1)*size(rot_res, 2), size(rot_res, 3)])';
        tran_res = reshape(tran_res,...
            [size(tran_res, 1)*size(tran_res, 2), size(tran_res, 3)])';

        d_mean_traj = d_mean_traj +...
            [dtw_c(rot_init, rot_res), dtw_c(tran_init, tran_res)];
    end
end

% Take average over the whole trajectory
d_mean_traj = d_mean_traj/(n_demo * n_sample * n_step);