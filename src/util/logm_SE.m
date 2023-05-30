function xi_hat = logm_SE(g)
% expm_SE Closed-form logarithm from SE(3)/SE(2) into se(3)/se(2)
%
% Input
%   g: homogeneous transformation matrix for SE(3)/SE(2)
%
% Output
%   xi_hat: closed-form matrix logarithm of SE(3)/SE(2)
%
% Author
%   Sipu Ruan, 2020
%
% See also
%   logm_SO, jacobian_inv_SO_l, vex

R = g(1:end-1,1:end-1);
t = g(1:end-1,end);

xi_hat = [logm_SO(R), jacobian_inv_SO_l( vex(logm_SO(R)) ) * t;...
    zeros(1,size(g,2))];