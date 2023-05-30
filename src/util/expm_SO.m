function R = expm_SO(w)
% expm_SO Closed-form exponential for so(3)/so(2) into SO(3)/SO(2)
%
% Input
%   w: \omega vector for rotation
%
% Output
%   R: rotation matrix
%
% Author
%   Sipu Ruan, 2020

dim = ( 1+sqrt(1+8*size(w,1)) )/2;

if norm(w) < 1e-15
    R = eye(dim);
    return;
end
    
R = eye(dim) + sin(norm(w)) / norm(w) * skew(w) +...
    (1-cos(norm(w)))/norm(w)^2 * skew(w)^2;