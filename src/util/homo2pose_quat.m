function pose = homo2pose_quat(g)
% homo2pose_quat Convert SE(3) homogenuous matrix into pose, rotation
% represented in Quaternion with order (x, y, z, w)
%
% Author
%   Sipu Ruan, 2022

quat = rotm2quat(g(1:3,1:3));
pose = [g(1:3,4)', quat(2:4), quat(1)];