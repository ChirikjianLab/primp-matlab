function g = expm_SE(xi)
% expm_SE3 Closed-form exponential from se(3) into SE(3)
%
% Input
%   xi: element in se(3)/se(2) of the form [\omega, v] or
%       [\hat{\omega}, v; zeros]
%
% Output
%   g: homogeneous transformation matrix for SE(3)/SE(2)
%
% Author
%   Sipu Ruan, 2020
%
% See also
%   expm_SO, jacobian_SO_l, vex

if size(xi,1) == 4 && size(xi,2) == 4
    xi_vex = [ vex(xi(1:3,1:3)); xi(1:3,4) ];
    xi = xi_vex;

elseif size(xi,1) == 3 && size(xi,2) == 3
    xi_vex = [ xi(2,1); xi(1:2,3) ];
    xi = xi_vex;
end

if length(xi) == 3
    g = [expm_SO(xi(1)), jacobian_SO_l(xi(1))*xi(2:3); 0,0,1];
elseif length(xi) == 6
    g = [expm_SO(xi(1:3)), jacobian_SO_l(xi(1:3))*xi(4:6); 0,0,0,1];
end