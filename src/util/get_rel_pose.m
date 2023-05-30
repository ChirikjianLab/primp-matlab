function g_rel = get_rel_pose(g_1, g_2, group_name)
% get_rel_pose Get relative poses for SE(3)/PCG(3).
%
% Input
%   g_1, g_2  : Two poses
%   group_name: Name of the group
%
% Output
%   g_rel     : Relative pose
%
% Author
%   Sipu Ruan, 2022

g_rel = eye(4);

if strcmp(group_name, 'SE')
    g_rel = g_1 \ g_2;
elseif strcmp(group_name, 'PCG')
    g_rel(1:3,1:3) = g_1(1:3,1:3) \ g_2(1:3,1:3);
    g_rel(1:3,4) = g_2(1:3,4) - g_1(1:3,4);
end

end