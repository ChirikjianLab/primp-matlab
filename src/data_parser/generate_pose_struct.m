function g_struct = generate_pose_struct(g_matrix, group_name)
% generate_pose_struct Construct pose structure with homogeneous matrix and
% pose in cartesian and exponential coordinates
%
%  Input
%    g_matrix  : Cell array that stores 4x4 pose matrices
%    group_name: Name of the Lie group
%
%  Output
%    g_struct  : Structure storing matrix, pose and exponential coordinates
%                 of the poses. Pose: [x,y,z,qx,qy,qz,qw]
%
%  Author
%    Sipu Ruan, 2022

n_traj = length(g_matrix);
n_step = size(g_matrix{1}, 3);

g_struct = cell(n_traj, 1);
for i = 1:n_traj
    g_struct{i}.matrix = g_matrix{i};
    
    g_struct{i}.pose = nan(7,n_step);
    g_struct{i}.exponential = nan(6,n_step);
    for j = 1:n_step
        g = g_struct{i}.matrix(:,:,j);
        g_struct{i}.pose(:,j) = homo2pose_quat(g)';
        g_struct{i}.exponential(:,j) = get_exp_coord(g, group_name);
    end
end
end