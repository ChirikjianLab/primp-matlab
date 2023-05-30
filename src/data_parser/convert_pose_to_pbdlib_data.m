function [Data, s] = convert_pose_to_pbdlib_data(g_demo_struct)
% convert_pose_to_pbdlib_data Convert pose data to format that is 
% compatible to PbD-lib
%
%  Input
%    g_demo_struct: Structure that stores demonstration poses
%
%  Output
%    Data         : Data arrary used in PbD-lib
%    s            : Structure storing viewing frame info
%
%  Author
%    Sipu Ruan, 2022

n_demo = length(g_demo_struct);
n_step = size(g_demo_struct{1}.pose, 2);
dt = 1 / (n_step-1);
dim = 7;

% Store all demo data into a long array
num = 0;
Data = nan(1+dim, 1, n_step*n_demo);
for i = 1:n_demo        
    for j = 1:n_step
        num = num+1;
        Data(1, 1, num) = (j-1) * dt;
        Data(2:dim+1, 1, num) = g_demo_struct{i}.pose(1:dim,j);
    end
end

% Store data based on viewing frame
s = struct([]);

for i = 1:n_demo
    s(i).p.A = eye(dim+1);
    s(i).p.b = zeros(dim+1,1);

    for j = 1:n_step
        s(i).Data0(1, j) = (j-1) * dt;
        s(i).Data0(2:dim+1, j) = g_demo_struct{i}.pose(1:dim,j);
    end
    
    s(i).nbData = n_step;
    s(i).Data = s(i).Data0;
end