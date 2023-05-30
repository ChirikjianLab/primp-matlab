function J_l = jacobian_SO_l(w)
% jacobian_SO3_l Closed-form left Jacobian of SO(3)
%
% Input
%   w: \omega vector for the rotation
%
% Output
%   J_l: Closed-form left Jacobian of SO(3)/SO(2)
%
% Author
%   Sipu Ruan, 2020
%
% See also
%   skew

dim = ( 1+sqrt(1+8*size(w,1)) )/2;  

if norm(w) < 1e-15
    J_l = eye(dim);
    return;
end

J_l = eye(dim) + (1-cos(norm(w)))/norm(w)^2 * skew(w) + ...
    (norm(w) - sin(norm(w)))/norm(w)^3 * skew(w)^2;