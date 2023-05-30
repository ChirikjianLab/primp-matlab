function distance = get_dist_traj(traj1, traj2, opt)
% get_dist_traj Compute the distance between two trajectories
%
% Input
%   traj1, traj2: Two trajectories as homogenuous transformation matrices
%   opt         : Option for distance measure
%
% Output
%   distance    : Structure with separated translation and rotation
%                  distances
%
% Author
%   Sipu Ruan, 2022

distance.translation = 0;
distance.rotation = 0;

if nargin < 3 || strcmp(opt, 'ed')
    if size(traj1,3) ~= size(traj2,3)
        error('Number of points in the two trajectories should be the same!')
    end
    
    for i = 1:size(traj1,3)
        % Translation part
        distance.translation = distance.translation +...
            norm( traj1(1:3,4,i) - traj2(1:3,4,i) );
        
        % Rotation part
        rot_diff_hat = logm_SO(traj1(1:3,1:3,i)' * traj2(1:3,1:3,i));
        distance.rotation = distance.rotation +...
            norm( [-rot_diff_hat(2,3); rot_diff_hat(1,3);
            -rot_diff_hat(1,2)] );
    end
else
    error('Not a supported distance measure!')
end

end