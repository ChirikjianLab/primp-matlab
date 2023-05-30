function exp_coord = get_exp_coord(g, group_name)
% get_exp_coord Get exponential coordinates from Lie group to Lie algebra
%  
% Input
%   g: homogeneous transformation matrix for poses
%   group_name: Name of the group, supports 'SO', 'SE' and 'PCG'
%
% Output
%   exp_coord: exponential coordinates of the form [\omega, v]
%
% Author
%   Sipu Ruan, 2020
%
% See also
%   logm_SE, logm_SO

if (nargin == 1) || strcmp(group_name, 'SE')
    % Special Euclidean group
    X_exp = logm_SE(g);
    
    if size(X_exp,1) == 3
        exp_coord = [X_exp(2,1); X_exp(1:2,3)];
        
    elseif size(X_exp,1) == 4
        exp_coord = [-X_exp(2,3); X_exp(1,3); -X_exp(1,2); X_exp(1:3,4)];
    end
    
elseif strcmp(group_name, 'SO')
    % Special orthogonal group
    X_exp = logm_SO(g);
    
    if size(X_exp,1) == 2
        exp_coord = X_exp(2,1);
        
    elseif size(X_exp,1) == 3
        exp_coord = [-X_exp(2,3); X_exp(1,3); -X_exp(1,2)];
    end
    
elseif strcmp(group_name, 'PCG')
    % Pose change group
    X_exp = logm_SE(g);
    
    if size(X_exp,1) == 3
        exp_coord = [X_exp(2,1); g(1:2,3)];
        
    elseif size(X_exp,1) == 4
        exp_coord = [-X_exp(2,3); X_exp(1,3); -X_exp(1,2); g(1:3,4)];
    end

else
    error('Group not supported! Currently supports SO, SE and PCG.')
end