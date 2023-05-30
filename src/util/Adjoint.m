function Ad = Adjoint(g, group_name)
% Adjoint Adjoint operator of Lie group. Currently support: SE(2)/SE(3), 
% PCG(3)
%
% Input
%   g         : group element
%   group_name: The name of the group
%
% Output
%   Ad        : Adjoint of a group element
%
% Author
%   Sipu Ruan, 2022

if strcmp(group_name, 'SE')
    if size(g,1) == 3
        M = [0, 1; -1, 0];
        Ad = [1, 0, 0; M*g(1:2,3), g(1:2,1:2)];
    elseif size(g,1) == 4
        R = g(1:3,1:3);
        T = [0, -g(3,4), g(2,4); g(3,4), 0, -g(1,4); -g(2,4), g(1,4), 0];
        Ad = [R, zeros(3,3); T*R, R];
    else
        error('Incorrect dimension');
    end
    
elseif strcmp(group_name, 'PCG')
    if size(g,1) == 4
        R = g(1:3,1:3);
        Ad = [R, zeros(3,3); zeros(3,3), eye(3)];
    else
        error('Incorrect dimension');
    end
    
else
    error('Group not supported.')
end