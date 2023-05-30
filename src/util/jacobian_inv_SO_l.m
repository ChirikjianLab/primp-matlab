function J_l_inv = jacobian_inv_SO_l(w)
% jacobian_inv_SO_l Closed-form left inverse Jacobian of SO(3)/SO(2)
%
% Input
%   w: \omega vector for the rotation
%
% Output
%   J_l_inv: Closed-form left inverse Jacobian of SO(3)/SO(2)
%
% Author
%   Sipu Ruan, 2020
%
% See also
%   skew

dim = ( 1+sqrt(1+8*size(w,1)) )/2;

if norm(w) < 1e-15
    J_l_inv = eye(dim);
    return;
end

J_l_inv = eye(dim) - 1/2 * skew(w) + ...
    ( 1/norm(w)^2 - (1+cos(norm(w)))/(2*norm(w)*sin(norm(w))) ) * skew(w)^2;