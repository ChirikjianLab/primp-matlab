function d = dtw_legacy(s, t, argin)
% Dynamic time warping (DTW) of two signals
%
% Input
%   s    : Signal 1, size k*ns (k1*k2*ns), k is data dim, ns is time step
%   t    : Signal 2, size k*nt (k1*k2*nt), k is data dim, nt is time step
%   argin: Input arguments
%     w - Window parameter, if s(i) is matched with t(j) then |i-j|<=w
%     method - Distance metric
%
% Output
%   d    : Resulting distance
%
% Copyright (C) 2013 Quan Wang <wangq10@rpi.edu>,
% Signal Analysis and Machine Perception Laboratory,
% Department of Electrical, Computer, and Systems Engineering,
% Rensselaer Polytechnic Institute, Troy, NY 12180, USA
%
% URL: https://www.mathworks.com/matlabcentral/fileexchange/43156-dynamic-time-warping-dtw
%
% Modified by Sipu Ruan, 2023
%  - Customized distance metric
%  - Input 2D/3D matrix as data sequence

if nargin < 3
    w = inf;
    method = 'euclidean';
elseif ~isfield(argin, 'w')
    w = inf;
    method = argin.method;
elseif ~isfield(argin, 'method')
    w = argin.w;
    method = 'euclidean';
else
    w = argin.w;
    method = argin.method;
end

% Sequence length
if size(s,3) == 1
    ns = size(s,2);
    nt = size(t,2);
    if size(s,1) ~= size(t,1)
        error('Error in dtw(): dimensions not match.');
    end

else
    ns = size(s,3);
    nt = size(t,3);
    if size(s,1) ~= size(t,1) || size(s,2) ~= size(t,2)
        error('Error in dtw(): dimensions not match.');
    end
end

w = max(w, abs(ns-nt)); % adapt window size

%% initialization
D = zeros(ns+1, nt+1) + Inf; % cache matrix
D(1,1) = 0;

%% begin dynamic programming
for i = 1:ns
    for j = max(i-w,1):min(i+w,nt)
        if size(s,3) == 1
            oost = dist_metric( s(:,i), t(:,j), method );
        else
            oost = dist_metric( s(:,:,i), t(:,:,j), method );
        end

        D(i+1,j+1) = oost + min( [D(i,j+1), D(i+1,j), D(i,j)] );
    end
end
d = D(ns+1, nt+1);
end


function d = dist_metric(s, t, method)
d = inf;

if nargin < 3
    method = 'euclidean';
end

switch method
    case 'euclidean'
        % Euclidean distance
        d = norm(s-t);

    case 'so3'
        % Geodesic distance on SO(3)
        d = norm(logm_SO(s' * t));

    case 'se3'
        % Geodesic distance on SO(3)
        d = norm(get_exp_coord(s \ t));
end

d = d^2;
end