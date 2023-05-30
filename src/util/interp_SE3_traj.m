function x_interp = interp_SE3_traj(t0, x, t)
% interp_SE3_traj Interpolation for SE(3) trajectory using singular value
% decomposition
%
% Author
%   Thomas Mitchel 2018
%
% Dependency
%   svd, df_vect
%
% Reference
%   "An SVD-Based Projection Method for Interpolation on SE(3).
%    Calin Belta, Vijay Kumar. 2002."

num_t0 = length(t0);
num_traj = size(x, 2)/4;

% Pre-allocate array
x_interp = zeros(4, 4*num_traj, length(t));

% Pre-allocate weight: Inertia matrix for unit sphere of unit mass
I = (2/5)*eye(3);
J = (1/2)*trace(I)*eye(3) - I;

dX = df_vect(t0, x, 1, 2, 3);

% Column-wise interpolation in SE(3)
count = 0;
for i = 1:num_t0-1
    if i == num_t0-1
        intv = t(t0(i) <= t & t <= t0(i+1));
    else
        intv = t(t0(i) <= t & t < t0(i+1));
    end
    if ~isempty(intv)
        dt = t0(i+1) - t0(i);
        for j = 1:num_traj
            g0 = x(:, 4*(j-1)+1:4*j, i);
            g1 = x(:, 4*(j-1)+1:4*j, i+1);
            dg0 = dX(:, 4*(j-1)+1:4*j, i);
            dg1 = dX(:, 4*(j-1)+1:4*j, i+1);

            % Interp rotation & posititon
            dx = g1 - g0;
            dv = dg1 - dg0;

            M3 = 6*((dg0+dg1)/(dt^2)) - 12*(dx/(dt^3));
            M2 =(dv/dt) - M3*((t0(i) + t0(i+1))/2);
            M1 = dg0 - M3*((t0(i)^2)/2) - M2*t0(i);
            M0 = g0 - M3*((t0(i)^3)/6) - M2*((t0(i)^2)/2) - M1*t0(i);

            for l = 1:length(intv)
                g_itp = zeros(4, 4);
                t_eval = intv(l);
                M = M0 + M1*t_eval + (M2/2)*(t_eval^2) + (M3/6)*(t_eval^3);
                [U, ~, V] = svd(M(1:3, 1:3)*J);
                g_itp(1:3, 1:3) = U*V';
                g_itp(1:4, 4) = [M(1:3, 4);1];
                x_interp(:, 4*(j-1)+1:4*j, count + l) = g_itp;
            end
        end
        count = count + length(intv);
    end
end