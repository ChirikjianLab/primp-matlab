function w_hat = logm_SO(R)
% logm_SO Closed-form logarithm from SO(3)/SO(2) into so(3)/so(2)
%
% Input
%   R: rotation matrix
%
% Output
%   w_hat: \hat{\omega}, the skew symmetric matrix
%
% Author
%   Sipu Ruan, 2020

if size(R,1) == 2
    th = atan2(R(2,1), R(1,1));
    w_hat = [0, -th; th, 0];

elseif size(R,1) == 3
    theta = acos( (trace(R)-1)/2 );

    if sin(theta) < 1e-15
        w_hat = zeros(3,3);
        return;
    end

    w_hat = 1/2 * theta/sin(theta) * (R-R');
end