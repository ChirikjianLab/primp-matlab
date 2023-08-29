function [Dist, D, k, w] = dtw_lie_group(t, r, group_name)
% dtw_lie_group Dynamic Time Warping Algorithm for sequences of Lie group 
% elements
%
% Input
%  t is the vector you are testing against
%  r is the vector you are testing
%  group_name is the name of the Lie group, i.e., euclidean, so3, se3
%
% Output
%  Dist is unnormalized distance between t and r
%  D is the accumulated distance matrix
%  k is the normalizing factor
%  w is the optimal path
%  
% Reference
%  Timothy Felty (2023). Dynamic Time Warping (https://www.mathworks.com/matlabcentral/fileexchange/6516-dynamic-time-warping), MATLAB Central File Exchange. Retrieved August 29, 2023.
%
% Modified by Sipu Ruan, 2023
%  - Customized distance metric
%  - Input 2D/3D matrix as data sequence
%  - Reverse ordering of optimal path

% Sequence length
if size(t,3) == 1
    N = size(t,2);
    M = size(r,2);
    if size(t,1) ~= size(r,1)
        error('Error in dtw(): dimensions not match.');
    end

else
    N = size(t,3);
    M = size(r,3);
    if size(t,1) ~= size(r,1) || size(t,2) ~= size(r,2)
        error('Error in dtw(): dimensions not match.');
    end
end

% Construct distance matrix
d = zeros(N, M);
for n=1:N
    for m=1:M
        if size(t,3) == 1
            d(n,m) = dist_metric( t(:,n), r(:,m), group_name );
        else
            d(n,m) = dist_metric( t(:,:,n), r(:,:,m), group_name );
        end
    end
end

% d=(repmat(t(:),1,M)-repmat(r(:)',N,1)).^2; %this replaces the nested for loops from above Thanks Georg Schmitz

D=zeros(size(d));
D(1,1)=d(1,1);
for n=2:N
    D(n,1)=d(n,1)+D(n-1,1);
end
for m=2:M
    D(1,m)=d(1,m)+D(1,m-1);
end
for n=2:N
    for m=2:M
        D(n,m)=d(n,m)+min([D(n-1,m),D(n-1,m-1),D(n,m-1)]);
    end
end

Dist=D(N,M);
n=N;
m=M;
k=1;
w=[];
w(1,:)=[N,M];
while ((n+m)~=2)
    if (n-1)==0
        m=m-1;
    elseif (m-1)==0
        n=n-1;
    else
        [values,number]=min([D(n-1,m),D(n,m-1),D(n-1,m-1)]);
        switch number
            case 1
                n=n-1;
            case 2
                m=m-1;
            case 3
                n=n-1;
                m=m-1;
        end
    end
    k=k+1;
    w=cat(1,w,[n,m]);
end

w = flip(w);
end

%% Distance metric
function d = dist_metric(s, t, group_name)
d = inf;

if nargin < 3
    group_name = 'euclidean';
end

switch group_name
    case 'euclidean'
        % Euclidean distance
        d = norm(s-t)^2;

    case 'SO'
        % Geodesic distance on SO(3)
        d = norm(logm_SO(s' * t))^2;

    case 'SE'
        % Geodesic distance on SE(3)
        d = norm(get_exp_coord(s \ t))^2;
end
end