function d_desired_pose = evaluate_desired_pose(traj_res, g_desired, t_via)
% evaluate_desired_pose Evaluate similarity between result and desired
% pose. Distances for rotation and translation parts are separated.
%
% Input
%   traj_res      : Resulting trajectory samples
%   g_desired     : Desired pose
%   t_via         : Time step of the via pose
%
% Output
%   d_desired_pose: Distance metric to desired pose, [rot, tran]
%
% Author
%   Sipu Ruan, 2022

% Index of the desired time in the trajectory
n_step = size(traj_res{1}.matrix, 3);
step_via = floor(t_via * n_step);
if step_via <= 0
    step_via = 1;
elseif step_via >= n_step
    step_via = n_step;
end

% Distance between pose in trajectory and desired via pose
n_sample = length(traj_res);
d_desired_pose = [0, 0];

for j = 1:n_sample
    g_res = traj_res{j}.matrix(:,:,step_via);
    d_desired_pose = d_desired_pose +...
        [norm(logm_SO(g_desired(1:3,1:3)' * g_res(1:3,1:3))),...
        norm(g_desired(1:3,4)-g_res(1:3,4))];
end
d_desired_pose = d_desired_pose/n_sample;

end