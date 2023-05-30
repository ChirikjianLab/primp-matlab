function pose = homo2pose_axang(g)
% homo2pose_axang Convert homogeneous matrix to pose. Rotation
% parameterized as axis-angle pair
%
% Input
%   g   : pose in homogeneous matrix
%
% Output
%   pose: Pose vector with axis-angle parameteriztion for rotation
%
% Author
%   Sipu Ruan, 2022

pose = [g(1:3,4)', rotm2axang(g(1:3,1:3))];