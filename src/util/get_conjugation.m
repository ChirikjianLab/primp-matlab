function g_o = get_conjugation(g, h, group_name)
% get_conjugation Get conjugation operation for SE(3)/PCG(3). Conjugation
% switches a group element into a new reference frame.
%
% Input
%   g         : Pose in the original frame (global frame)
%   h         : New reference frame
%   group_name: Name of the group
%
% Output
%   g_o       : Pose after conjugation
%
% Author
%   Sipu Ruan, 2023
%
% Reference
%   Chirikjian, G.S., Mahony, R., Ruan, S. and Trumpf, J., 2018.
%   Pose changes from a different point of view. 
%   Journal of Mechanisms and Robotics, 10(2), p.021008.

g_o = eye(4);

if strcmp(group_name, 'SE')
    g_o = h * (g / h);
elseif strcmp(group_name, 'PCG')
    g_o(1:3,1:3) = h(1:3,1:3) * (g(1:3,1:3) / h(1:3,1:3));
    g_o(1:3,4) = h(1:3,1:3) * g(1:3,4);
end

end