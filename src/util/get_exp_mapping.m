function g = get_exp_mapping(exp_coord, group_name)
% get_exp_mapping Get the exponential mapping from Lie algebra
% (exponential coordinates) to Lie group
%
% Input
%   exp_coord: exponential coordinates of the form [\omega, v]
%   group_name: Name of the group, supports 'SO', 'SE' and 'PCG'
%
% Output
%   g: homogeneous transformation matrix
%
% Author
%   Sipu Ruan, 2020
%
% See also
%   expm_SE, expm_SO

if (nargin == 1) || strcmp(group_name, 'SE')
    % Special Euclidean group  
    g = expm_SE(exp_coord);
    
elseif strcmp(group_name, 'SO')
    % Special orthogonal group   
    g = expm_SO(exp_coord);
    
elseif strcmp(group_name, 'PCG')
    % Pose change group
    if size(exp_coord,1) == 3
        g = [expm_SO(exp_coord(1)), exp_coord(2:3); 0, 0, 0, 1];
        
    elseif size(exp_coord,1) == 6
        g = [expm_SO(exp_coord(1:3)), exp_coord(4:6); 0, 0, 0, 1];
    end
    
else
    error('Group not supported! Currently supports SO, SE and PCG.')
end