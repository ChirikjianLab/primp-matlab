function Sig12 = cov_SE_1_2(Sig1, Sig2, mu1, mu2)
% cov_SE_1_2 First-order approximation of convolution of two PDF in Lie
% group
%
% Input
%   Sig1, Sig2: Covariance matrix
%   mu1, mu2  : Mean pose
%
% Output
%   Sig12     : Covariance of convolution
%
% Author
%   Sipu Ruan, 2022

A = Adjoint(inv(mu2), 'SE') * Sig1 * Adjoint(inv(mu2), 'SE')';
B = Sig2;

Sig12 = A + B;